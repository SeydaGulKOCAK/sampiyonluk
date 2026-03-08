#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                          drone_interface.py                                  ║
║                      MAVROS ↔ ROS2 Sistemi Köprüsü                          ║
╚══════════════════════════════════════════════════════════════════════════════╝

GENEL AÇIKLAMA:
---------------
Bu node, ArduPilot uçuş kontrolcüsü (Pixhawk) ile ROS2 yazılım sistemimiz
arasındaki TEK iletişim noktasıdır. "Köprü" görevi görür.

    ┌─────────────┐    MAVROS    ┌──────────────────┐    ROS2    ┌──────────────┐
    │  ArduPilot  │◄────────────►│ drone_interface  │◄──────────►│  Sistemimiz  │
    │  (Pixhawk)  │   protokol  │    (bu dosya)    │  topic'ler │  (diğer node)│
    └─────────────┘             └──────────────────┘            └──────────────┘

NE YAPAR:
---------
  1. Pixhawk'tan GELEN bilgileri (konum, hız, durum) ROS2 topic'lerine yayınlar
  2. ROS2 sisteminden GELEN komutları (setpoint, mod) Pixhawk'a iletir
  3. RC kill-switch veya mod değişimi algılarsa PILOT OVERRIDE yayınlar

KOORDİNAT SİSTEMİ:
------------------
  ArduPilot → NED  (North-East-Down):  X=Kuzey, Y=Doğu,  Z=Aşağı
  ROS2      → ENU  (East-North-Up):    X=Doğu,  Y=Kuzey, Z=Yukarı
  MAVROS bu dönüşümü otomatik yapar, biz ENU kullanırız.

ÇALIŞTIRMA:
-----------
  ros2 run my_swarm_pkg drone_interface --ros-args -p drone_id:=1
  ros2 run my_swarm_pkg drone_interface --ros-args -p drone_id:=2
  ros2 run my_swarm_pkg drone_interface --ros-args -p drone_id:=3

ŞARTNAME KARŞILAŞTIRMASI:
  §5.3  Dağıtık mimari → Her drone kendi drone_interface'ini çalıştırır ✅
  §5.4  Failsafe       → pilot_override ile RC kaybı algılanır ✅
  §5.5.4 Kill-switch   → GUIDED dışı mod = pilot_override=True ✅
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Bool, String


class DroneInterface(Node):
    """
    MAVROS ile ROS2 sistemimiz arasındaki köprü node'u.
    Her drone için ayrı bir örnek çalışır.
    """

    def __init__(self):
        super().__init__('drone_interface')

        # ══════════════════════════════════════════════════════
        # PARAMETRE OKUMA
        # drone_id: Hangi drone? (1, 2 veya 3)
        # launch dosyasından veya -p drone_id:=2 ile verilir
        # ══════════════════════════════════════════════════════
        self.declare_parameter('drone_id', 1)
        self.drone_id = self.get_parameter('drone_id').value

        # Topic prefix oluştur: drone_id=2 → ns='drone2'
        self.ns = f'drone{self.drone_id}'
        self.mavros_ns = f'/{self.ns}/mavros'

        self.get_logger().info(
            f'🔌 drone_interface başlatılıyor...\n'
            f'   Drone ID  : {self.drone_id}\n'
            f'   Topic ns  : /{self.ns}/\n'
            f'   MAVROS ns : {self.mavros_ns}/'
        )

        # ══════════════════════════════════════════════════════
        # DURUM DEĞİŞKENLERİ
        # ══════════════════════════════════════════════════════
        self.current_state = State()   # MAVROS'tan gelen son durum
        self.last_mode = ''            # Son gönderilen mod (tekrar engeli)
        self.pilot_override_active = False  # RC kill-switch aktif mi?
        # State-triggered yayın için: son yayınlanan değeri sakla.
        # Böylece sadece değişince yayınlarız, gereksiz Wi-Fi trafiği önlenir.
        self._last_published_override = None

        # ══════════════════════════════════════════════════════
        # QoS AYARI
        # NEDEN BEST_EFFORT?
        # MAVROS bazı topic'leri BEST_EFFORT ile yayınlar.
        # Subscriber RELIABLE olursa mesajlar HİÇ GELMEZ!
        # (swarm_takeoff'taki QoS uyumsuzluk uyarısının sebebi buydu)
        # ══════════════════════════════════════════════════════
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ══════════════════════════════════════════════════════
        # MAVROS → SİSTEM ABONELİKLERİ
        # Pixhawk'tan okuyup sistemimize yayınlıyoruz
        # ══════════════════════════════════════════════════════

        # 1) KONUM: Her 50ms'de bir gelir (20 Hz)
        #    position.x/y/z = ENU koordinat (metre)
        #    orientation    = Quaternion (drone yönü)
        #    → formation_controller ve collision_avoidance kullanır
        self.create_subscription(
            PoseStamped,
            f'{self.mavros_ns}/local_position/pose',
            self._mavros_pose_cb,
            best_effort_qos   # MAVROS BEST_EFFORT yayınlıyor!
        )

        # 2) HIZ: Anlık hız vektörü (m/s)
        #    twist.linear.x/y/z = ENU hız
        #    → collision_avoidance APF'de "komşu nereye gidiyor?" için kullanır
        self.create_subscription(
            TwistStamped,
            f'{self.mavros_ns}/local_position/velocity_local',
            self._mavros_velocity_cb,
            best_effort_qos
        )

        # 3) DURUM: armed mi? hangi modda? bağlı mı?
        #    → Pilot override tespiti için izliyoruz
        self.create_subscription(
            State,
            f'{self.mavros_ns}/state',
            self._mavros_state_cb,
            10
        )

        # ══════════════════════════════════════════════════════
        # SİSTEM → MAVROS ABONELİKLERİ
        # Sistemimizden alıp Pixhawk'a iletiyoruz
        # ══════════════════════════════════════════════════════

        # 4) NİHAİ SETPOINT: collision_avoidance'dan gelir
        #    APF uygulandıktan sonra "gerçek hedef konum"
        #    → Pixhawk bu koordinata doğru uçar
        self.create_subscription(
            PoseStamped,
            f'/{self.ns}/setpoint_final',
            self._setpoint_final_cb,
            10
        )

        # 5) MOD KOMUTU: SADECE local_fsm yazar! (güvenlik kuralı)
        #    State değişimlerinde mod komutunu gönderir:
        #    LANDING state → 'LAND', SAFETY state → 'RTL', vb.
        self.create_subscription(
            String,
            f'/{self.ns}/cmd_mode',
            self._cmd_mode_cb,
            10
        )

        # 6) ARM/DISARM KOMUTU: SADECE local_fsm yazar! (güvenlik kuralı)
        #    Bool mesajı: True=ARM, False=DISARM
        #    Kullanım senaryoları:
        #      STANDBY→IDLE geçişi : ARM=True  → motorlar hazır
        #      DISARM_WAIT state   : ARM=False → motorlar durur
        #      REARM state         : ARM=True  → sürüye yeniden katılım
        #    NEDEN TOPIC? local_fsm ayrı bir process, arm() metodunu
        #    doğrudan çağıramaz. Topic üzerinden haberleşmek zorundayız.
        self.create_subscription(
            Bool,
            f'/{self.ns}/cmd_arm',
            self._cmd_arm_cb,
            10
        )

        # 7) HASSAS İNİŞ HEDEFİ: precision_landing'den gelir
        #    Kamerayla renk zone tespit edilince iniş koordinatı gelir
        #    LAND_ZONE state'inde aktif (ileride yazacağız)
        self.create_subscription(
            PoseStamped,
            f'/{self.ns}/landing_target',
            self._landing_target_cb,
            10
        )

        # ══════════════════════════════════════════════════════
        # PUBLISHER'LAR
        # ══════════════════════════════════════════════════════

        # Sisteme konum yayınla (ENU metre)
        # → formation_controller centroid hesabı için okur
        # → collision_avoidance APF için okur
        self.pose_pub = self.create_publisher(
            PoseStamped, f'/{self.ns}/pose', 10
        )

        # Sisteme hız yayınla (ENU m/s)
        # → collision_avoidance "komşu nereye gidiyor?" için okur
        self.velocity_pub = self.create_publisher(
            TwistStamped, f'/{self.ns}/velocity', 10
        )

        # Pilot override durumu
        # True  = RC kill-switch / GUIDED dışı mod → otonom devre dışı
        # False = Normal, sistem kontrolde
        # → local_fsm ve intent_coordinator izler
        self.pilot_override_pub = self.create_publisher(
            Bool, f'/{self.ns}/pilot_override', 10
        )

        # MAVROS'a setpoint gönder (drone buraya gitsin)
        # setpoint_final ve landing_target bu publisher'ı kullanır
        self.mavros_setpoint_pub = self.create_publisher(
            PoseStamped,
            f'{self.mavros_ns}/setpoint_position/local',
            10
        )

        # ══════════════════════════════════════════════════════
        # SERVİS İSTEMCİLERİ
        # Topic'lerden farklı: istek-cevap mantığı
        # Mod değiştirme ve ARM için servis çağrısı gerekir
        # ══════════════════════════════════════════════════════

        # Uçuş modunu değiştir (GUIDED, LAND, RTL...)
        self.set_mode_client = self.create_client(
            SetMode, f'{self.mavros_ns}/set_mode'
        )

        # Drone ARM / DISARM et
        # ARM=True  → motorlar hazır
        # ARM=False → motorlar durur (güvenli)
        self.arming_client = self.create_client(
            CommandBool, f'{self.mavros_ns}/cmd/arming'
        )

        # ══════════════════════════════════════════════════════
        # PERİYODİK GÖREV (Timer) - HEARTBEAT
        # State-triggered yaklaşım:
        #   → Durum değişince ANINDA yayınla (_mavros_state_cb içinde)
        #   → Değişmese bile her 5 sn'de bir "heartbeat" gönder
        #     (Yeni başlayan node'lar mevcut durumu öğrensin)
        # NEDEN 2 Hz DEĞİL?
        #   3 drone × 2 Hz = saniyede 6 gereksiz mesaj → Wi-Fi trafiği
        #   Sürü büyüdükçe (10 drone) bu 20 mesaj/sn olur → darboğaz!
        # ══════════════════════════════════════════════════════
        self.create_timer(5.0, self._heartbeat_pilot_override)

        self.get_logger().info(
            f'✅ [{self.ns}] drone_interface HAZIR! Topic\'ler dinleniyor...'
        )

    # ══════════════════════════════════════════════════════════════════════
    # MAVROS → SİSTEM CALLBACK'LERİ
    # ══════════════════════════════════════════════════════════════════════

    def _mavros_pose_cb(self, msg: PoseStamped):
        """
        Pixhawk'tan gelen konumu sisteme ilet.
        MAVROS zaten NED→ENU dönüşümünü yapıyor, biz sadece iletiyoruz.
        """
        msg.header.frame_id = 'map'
        self.pose_pub.publish(msg)

    def _mavros_velocity_cb(self, msg: TwistStamped):
        """Pixhawk'tan gelen hızı sisteme ilet."""
        self.velocity_pub.publish(msg)

    def _mavros_state_cb(self, msg: State):
        """
        Pixhawk durum değişimlerini izle → Pilot override kontrolü yap.

        ÇALIŞMA MANTIĞI:
        Drone ARM'lıyken GUIDED dışı bir moda geçerse
        → Pilot RC ile müdahale etti demektir
        → pilot_override = True
        → intent_coordinator bu drone'u FLYING listesinden çıkarır
        → "Drone düştü!" yanlış alarmı engellenir (Şartname §5.5.4)
        """
        prev_mode = self.current_state.custom_mode
        self.current_state = msg

        # Mod değiştiyse logla
        if prev_mode != msg.custom_mode:
            self.get_logger().info(
                f'🔄 [{self.ns}] Mod: {prev_mode or "?"} → {msg.custom_mode}'
            )

        # Otonom sistem tarafından kullanılan izinli modlar (whitelist).
        # BRAKE : ArduPilot acil frenleme modu → sistemimiz tetikleyebilir
        # AUTO  : Önceden yüklenmiş rota modu  → sistemimiz tetikleyebilir
        # Bu modlar whitelist'te olmasa "pilot müdahalesi" sanılır ve
        # drone sürüden koparılır! (Yanlış alarm)
        AUTONOMOUS_MODES = ('GUIDED', 'LAND', 'RTL', 'BRAKE', 'AUTO')

        # ARM'lıyken whitelist dışı mod → GERÇEK PILOT MÜDAHALESİ!
        if msg.armed and msg.custom_mode not in AUTONOMOUS_MODES:
            if not self.pilot_override_active:
                self.get_logger().warn(
                    f'⚠️  [{self.ns}] PILOT OVERRIDE! Mod: {msg.custom_mode}'
                )
                self.pilot_override_active = True
                # Değişti → ANINDA yayınla (state-triggered)
                self._publish_pilot_override_state()
        else:
            if self.pilot_override_active:
                self.get_logger().info(
                    f'✅ [{self.ns}] Override bitti, otonom moda dönüldü'
                )
                self.pilot_override_active = False
                # Değişti → ANINDA yayınla (state-triggered)
                self._publish_pilot_override_state()

    # ══════════════════════════════════════════════════════════════════════
    # SİSTEM → MAVROS CALLBACK'LERİ
    # ══════════════════════════════════════════════════════════════════════

    def _setpoint_final_cb(self, msg: PoseStamped):
        """
        collision_avoidance → setpoint_final → Pixhawk.

        Pipeline:
          formation_controller → setpoint_raw (ideal hedef)
               ↓
          collision_avoidance  → setpoint_final (APF sonrası gerçek hedef)
               ↓
          drone_interface (biz) → MAVROS → Pixhawk → drone uçar!
        """
        self.mavros_setpoint_pub.publish(msg)

    def _cmd_mode_cb(self, msg: String):
        """
        local_fsm'den gelen mod komutunu Pixhawk'a ilet.

        KURAL: SADECE local_fsm bu topic'e yazar!
        Başka node yazarsa kaos olur. local_fsm tüm state geçişlerini yönetir.

        Örnek akışlar:
          state=FLYING  → cmd_mode='GUIDED' → Otonom uçuş
          state=LANDING → cmd_mode='LAND'   → Pixhawk iner
          state=SAFETY  → cmd_mode='RTL'    → Eve dön
        """
        if msg.data == self.last_mode:
            return  # Zaten bu moddayız, tekrar gönderme

        self.get_logger().info(
            f'🧭 [{self.ns}] Mod komutu: {msg.data}'
        )
        self.last_mode = msg.data

        if self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = msg.data
            requested_mode = msg.data  # closure için kopyala

            # async çağrı: node bloklanmaz, cevap gelince callback çalışır.
            future = self.set_mode_client.call_async(req)

            # NEDEN FUTURE DİNLEMELİYİZ?
            # Pixhawk isteği reddedebilir: yetersiz GPS, düşük batarya,
            # sensör hatası vb. Yanıt dinlenmezse sistemimiz başarılı
            # sandığı için setpoint göndermeye devam eder → tehlikeli!
            def _on_set_mode_done(f):
                try:
                    result = f.result()
                    if result.mode_sent:
                        self.get_logger().info(
                            f'✅ [{self.ns}] Mod değişti: {requested_mode}'
                        )
                    else:
                        # Pixhawk reddetti! last_mode'u sıfırla ki
                        # bir sonraki komutta tekrar denensin.
                        self.last_mode = ''
                        self.get_logger().error(
                            f'❌ [{self.ns}] Pixhawk modu reddetti: '
                            f'{requested_mode} → GPS/batarya/sensör sorunu olabilir!'
                        )
                except Exception as e:
                    self.last_mode = ''
                    self.get_logger().error(
                        f'❌ [{self.ns}] set_mode servis hatası: {e}'
                    )

            future.add_done_callback(_on_set_mode_done)
        else:
            self.get_logger().warn(
                f'⚠️  [{self.ns}] set_mode servisi hazır değil! '
                f'MAVROS bağlı mı?'
            )

    def _cmd_arm_cb(self, msg: Bool):
        """
        local_fsm'den gelen ARM/DISARM komutunu uygula.
        msg.data = True  → ARM   (motorlar hazır, uçuşa izin)
        msg.data = False → DISARM (motorlar dur, güvenli park)

        Çağrıldığı state geçişleri:
          STANDBY → IDLE        : ARM=True
          FLYING → DISARM_WAIT  : ARM=False (iniş sonrası)
          REARM state           : ARM=True  (sürüye yeniden katılım)
        """
        self.arm(value=msg.data)

    def _landing_target_cb(self, msg: PoseStamped):
        """
        precision_landing'den gelen hassas iniş hedefini Pixhawk'a ilet.
        LAND_ZONE state'inde aktif olur.
        Şartname: Tolerans dışı iniş = 0 puan! Bu yüzden hassas iniş kritik.
        """
        self.mavros_setpoint_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════════
    # PERİYODİK GÖREV
    # ══════════════════════════════════════════════════════════════════════

    def _publish_pilot_override_state(self):
        """
        pilot_override değeri DEĞİŞTİĞİNDE anında yayınla.
        _mavros_state_cb tarafından çağrılır.

        State-triggered yaklaşım:
          - Değer True→False veya False→True olduğunda ANINDA yayınla
          - Wi-Fi'ya gereksiz trafik bindirmez
          - Değişim anında hemen tepki verir (gecikme yok!)
        """
        if self.pilot_override_active != self._last_published_override:
            self._last_published_override = self.pilot_override_active
            msg = Bool()
            msg.data = self.pilot_override_active
            self.pilot_override_pub.publish(msg)

    def _heartbeat_pilot_override(self):
        """
        Her 5 saniyede bir mevcut durumu yayınla (heartbeat).

        NEDEN GEREKLİ?
        Yeni başlayan bir node (örn. local_fsm restart oldu) mevcut
        pilot_override durumunu bilmez. Heartbeat sayesinde en fazla
        5 saniye içinde güncel durumu öğrenir.

        5 sn seçimi:
          - 2 Hz (0.5sn) → 3 drone = 6 mesaj/sn → gereksiz trafik
          - 5 sn         → 3 drone = 0.6 mesaj/sn → kabul edilebilir
        """
        msg = Bool()
        msg.data = self.pilot_override_active
        self.pilot_override_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════════
    # YARDIMCI FONKSİYONLAR
    # ══════════════════════════════════════════════════════════════════════

    def arm(self, value: bool = True):
        """
        Drone ARM/DISARM et.
        ARM=True  → Motorlar çalışmaya hazır
        ARM=False → Motorlar durur (güvenli)
        Şartname: Tüm drone'lar iniş yapıp DISARM olunca görev tamamdır.
        """
        if self.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = value
            action = 'ARM' if value else 'DISARM'
            self.get_logger().info(
                f'🔐 [{self.ns}] {action} komutu gönderiliyor...'
            )
            future = self.arming_client.call_async(req)

            # Pixhawk ARM/DISARM isteğini kabul etti mi?
            # Reddederse (örn: öncalibrasyon eksik) logluyoruz.
            def _on_arm_done(f):
                try:
                    result = f.result()
                    if result.success:
                        self.get_logger().info(
                            f'✅ [{self.ns}] {action} başarılı!'
                        )
                    else:
                        self.get_logger().error(
                            f'❌ [{self.ns}] Pixhawk {action} reddetti! '
                            f'Kalibrasyon/güvenlik kontrolü gerekli olabilir.'
                        )
                except Exception as e:
                    self.get_logger().error(
                        f'❌ [{self.ns}] arming servis hatası: {e}'
                    )

            future.add_done_callback(_on_arm_done)
        else:
            self.get_logger().warn(f'⚠️  [{self.ns}] arming servisi hazır değil!')


# ══════════════════════════════════════════════════════════════════════════
# ANA FONKSİYON
# ══════════════════════════════════════════════════════════════════════════

def main(args=None):
    """
    Node başlatma.

    TEST ETME:
    ----------
    1) Simülasyon çalışıyorken (Gazebo + SITL + MAVROS açıkken):
       ros2 run my_swarm_pkg drone_interface --ros-args -p drone_id:=1

    2) Topic'leri kontrol et (başka terminalde):
       ros2 topic echo /drone1/pose
       ros2 topic echo /drone1/pilot_override
       ros2 topic list | grep drone1

    3) Konum geliyor mu?
       ros2 topic hz /drone1/pose   (→ yaklaşık 20 Hz olmalı)
    """
    rclpy.init(args=args)
    node = DroneInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'⛔ drone_interface durduruluyor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
