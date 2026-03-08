#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                       intent_coordinator.py                                  ║
║              Sürü Koordinatörü — Lider İHA'nın Kararname Üreticisi          ║
╚══════════════════════════════════════════════════════════════════════════════╝

GENEL AÇIKLAMA:
---------------
Bu node tüm sürünün "ne yapacağına" karar verir — ancak SADECE o an lider
olan drone'da bu kararı /swarm/intent topic'ine yayınlar.
Lider olmayan drone'lar dinler, seq takibi yapar ve yeni lider olmaya hazırlanır.

MİMARİ (§5.3 Dağıtık Kural):
------------------------------
    Her İHA'da çalışır, ama SADECE lider olan /swarm/intent YAZAR.
    GCS bu topic'e KESİNLİKLE YAZMAZ → mode gating ile garanti altına alındı.
    Lider olmayan drone'lar gözlemler + lider seçimine hazır bekler.

LİDER SEÇİMİ — BULLY ALGORİTMASI (Basitleştirilmiş):
------------------------------------------------------
    1) Her drone diğerlerinin /local_state mesajlarını heartbeat olarak izler
    2) FLYING veya REJOIN state'indeki canlı drone'lar aday sayılır
    3) min(aday_drone_id'leri) = yeni lider
    4) Tüm drone'lar aynı hesaplamayı yapınca AYNI sonuca ulaşır → tutarlılık
    5) Merkezi seçim mekanizmasına gerek yok → tam dağıtık ✅

    Özel durum — görev başlangıcı (ARMING fazı):
    Henüz kimse FLYING değilse, STANDBY/IDLE olanlar arasından min(id) seçilir.
    Bu sayede kalkış öncesi de bir lider var ve IDLE intent gönderilebilir.

HEARTBEAT MEKANİZMASI:
----------------------
    local_fsm → 100ms (10 Hz) /drone_{id}/local_state yayınlar
    intent_coordinator → 600ms timeout (6 paket boşluğu)

    Neden 600ms > 500ms?
      600ms timeout ve 100ms yayın → 6 paket kaybolabilir, 7.'de alarm.
      Wi-Fi jitter (10-50ms) ile birkaç paket kaybolsa bile FALSE ALARM olmaz.

SEQ SÜREKLİLİĞİ (Kritik!):
----------------------------
    Tüm drone'lar (lider olmayan dahil) /swarm/intent dinler ve son seq'i saklar.
    Lider değiştiğinde yeni lider: _my_intent_seq = _global_last_seq + 1
    Bu sayede local_fsm'deki çift filtre yanlış DROP yapmaz.
    (Eski lider seq=50 gönderdi, yeni lider seq=51'den başlar → filtre geçer)

KAMERA DRONE KORUMASI:
-----------------------
    Şartname: Görev boyunca en az bir İHA'da kamera olmalı.
    Detach edilecek drone kamera drone'u olmamalı → QR algılama kesilmez.
    QR'da kamera drone'u yazıyorsa → başka FLYING drone otomatik seçilir.

ŞARTNAME KARŞILAŞTIRMASI:
  §5.3   Dağıtık mimari    → Sadece lider yazar, GCS yazamaz ✅
  §5.5   Failsafe          → 600ms → lider değişimi → yeni lider ≤1 intent süresi ✅
  §5.5.4 Kill-switch       → PILOT_OVERRIDE drone sürüden çıkarılır ✅
  §13    Renk zone'ları    → precision_landing koordinatları saklanır ✅
  §15    Birey çıkarma     → Kamera drone korunur, başkası detach edilir ✅
  §15    Birey ekleme      → En dolu bataryalı yedek drone çağrılır ✅

ÇALIŞTIRMA:
-----------
  ros2 run my_swarm_pkg intent_coordinator --ros-args -p drone_id:=1
  ros2 run my_swarm_pkg intent_coordinator --ros-args -p drone_id:=2
  ros2 run my_swarm_pkg intent_coordinator --ros-args -p drone_id:=3
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import UInt8, Bool
from geometry_msgs.msg import Point

from swarm_msgs.msg import (
    LocalState, SwarmIntent, JoinRequest,
    FormationCmd, QRResult, TaskTrigger,
)


# ══════════════════════════════════════════════════════════════════════════════
# DRONE DURUM SABİTLERİ
# local_fsm ile AYNI değerler — bağımlılık yaratmamak için burada kopyalanır.
# İleride ortak bir constants.py'ye taşınabilir.
# ══════════════════════════════════════════════════════════════════════════════
class DroneState:
    STANDBY        = 'STANDBY'
    IDLE           = 'IDLE'
    FLYING         = 'FLYING'
    DETACH         = 'DETACH'
    LAND_ZONE      = 'LAND_ZONE'
    DISARM_WAIT    = 'DISARM_WAIT'
    REARM          = 'REARM'
    REJOIN         = 'REJOIN'
    RETURN_HOME    = 'RETURN_HOME'
    LANDING        = 'LANDING'
    SAFETY_HOLD    = 'SAFETY_HOLD'
    PILOT_OVERRIDE = 'PILOT_OVERRIDE'

    # Lider adayı olabilecek state'ler
    # REJOIN da dahil: sürüye katılmakta olan drone da lider seçilebilir
    LEADER_ELIGIBLE = frozenset({FLYING, REJOIN})

    # "Havada" sayılan state'ler (active_drone_count için)
    # DETACH, LAND_ZONE: henüz yerdeymiş gibi sayılmaz
    AIRBORNE = frozenset({FLYING, DETACH, LAND_ZONE, REJOIN, RETURN_HOME})

    # "Devre dışı" state'ler → lider adayı olamaz, aktif sayılmaz
    INACTIVE = frozenset({DISARM_WAIT, LANDING, SAFETY_HOLD, PILOT_OVERRIDE})


# ══════════════════════════════════════════════════════════════════════════════
# GÖREV FAZI SABİTLERİ
# intent_coordinator'ın iç koordinasyon durumu.
# SwarmIntent.task_id'den FARKLIDIR:
#   task_id     → Her drone'a ne yapacağını söyler (dışa gönderilen komut)
#   MissionPhase → Lider drone'un iç koordinasyon adımı (içeride tutulan durum)
# ══════════════════════════════════════════════════════════════════════════════
class MissionPhase:
    IDLE       = 'IDLE'        # Görev başlamadı
    ARMING     = 'ARMING'      # Drone'lar ARM ediliyor (kalkış öncesi)
    NAVIGATING = 'NAVIGATING'  # Sonraki QR noktasına gidiliyor
    AT_QR      = 'AT_QR'       # QR noktasına gelindi, içeriği işleniyor
    DETACHING  = 'DETACHING'   # Bir drone sürüden ayrılıyor
    REJOINING  = 'REJOINING'   # Yedek drone sürüye katılıyor
    RETURNING  = 'RETURNING'   # Tüm QR'lar bitti, eve dönüş
    COMPLETE   = 'COMPLETE'    # Görev tamamen bitti


class IntentCoordinator(Node):
    """
    Sürü koordinatörü: lider seçimi, QR parse, SwarmIntent üretimi.
    Her drone'da çalışır; sadece lider olanı aktif olarak intent yayınlar.
    """

    # ── HEARTBEAT TIMEOUT ──────────────────────────────────────────────────
    # local_fsm 100ms (10 Hz) yayın yapıyor.
    # 600ms timeout → 6 paket boşluğu → Wi-Fi jitter güvenli marjı.
    # NEDEN 600ms? (500ms değil)
    #   local_fsm publish_rate = 100ms
    #   Güvenli margin = timeout / publish_rate = 600/100 = 6 paket
    #   → 6 paketten az kayıp → FALSE ALARM yok
    HEARTBEAT_TIMEOUT_MS: float = 600.0

    def __init__(self):
        super().__init__('intent_coordinator')

        # ══════════════════════════════════════════════════════
        # PARAMETRE OKUMA
        # ══════════════════════════════════════════════════════
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('num_drones', 3)
        # Kamera olan drone ID'si — detach'tan korunur!
        self.declare_parameter('camera_drone_id', 3)

        self.drone_id        = self.get_parameter('drone_id').value
        self.num_drones      = self.get_parameter('num_drones').value
        self.camera_drone_id = self.get_parameter('camera_drone_id').value
        self.ns = f'drone{self.drone_id}'

        self.get_logger().info(
            f'🧭 intent_coordinator başlatılıyor...\n'
            f'   Drone ID     : {self.drone_id}\n'
            f'   Toplam drone : {self.num_drones}\n'
            f'   Kamera drone : drone{self.camera_drone_id}'
        )

        # ══════════════════════════════════════════════════════
        # DRONE TAKİP TABLOSU
        # Her drone için state, son heartbeat zamanı ve canlılık.
        #
        # _drone_state[id]   → 'FLYING', 'STANDBY' vs.
        # _drone_hb_ns[id]   → Son heartbeat zamanı (nanosaniye, ROS clock)
        #                       0 = henüz hiç mesaj gelmedi
        # _drone_hb_seq[id]  → Son LocalState seq numarası (tekrar DROP için)
        # _drone_seen[id]    → İlk heartbeat geldi mi? (False = henüz yok)
        # ══════════════════════════════════════════════════════
        self._drone_state:  dict = {}
        self._drone_hb_ns:  dict = {}
        self._drone_hb_seq: dict = {}
        self._drone_seen:   dict = {}

        for i in range(1, self.num_drones + 1):
            self._drone_state[i]  = DroneState.STANDBY
            self._drone_hb_ns[i]  = 0      # 0 → henüz görülmedi
            self._drone_hb_seq[i] = 0
            self._drone_seen[i]   = False

        # ══════════════════════════════════════════════════════
        # LİDER SEÇİMİ DEĞİŞKENLERİ
        # ══════════════════════════════════════════════════════
        # Hesaplanan mevcut lider ID'si (0 = belirsiz/yok)
        self._leader_id: int  = 0
        # Ben lider miyim? (leader_id == drone_id)
        self._is_leader: bool = False
        # Önceki döngüde lider miydim? (transition tespiti için)
        self._was_leader: bool = False

        # ══════════════════════════════════════════════════════
        # SEQ TAKİBİ — KRİTİK!
        #
        # TÜM drone'lar (lider olmayan dahil) bu değeri takip eder.
        # Lider değişiminde yeni lider:
        #   _my_intent_seq = _global_last_seq + 1
        # Bu sayede local_fsm'deki çift filtre yanlış DROP yapmaz.
        # ══════════════════════════════════════════════════════
        # Ağda gördüğüm en büyük /swarm/intent seq'i
        self._global_last_seq: int = 0
        # Benim yayınladığım seq (sadece lider olduğumda artar)
        self._my_intent_seq: int = 0

        # ══════════════════════════════════════════════════════
        # GÖREV DURUMU
        # ══════════════════════════════════════════════════════
        self._task_active:   bool = False
        self._mission_phase: str  = MissionPhase.IDLE
        self._current_qr_id: int  = 1     # Şu anki hedef QR no
        self._last_qr_seq:   int  = 0     # Son işlenen QR (tekrar engeli)
        self._all_qr_done:   bool = False  # Tüm QR'lar işlendi mi?

        # ── AKTİF INTENT PARAMETRELERİ ────────────────────────
        # Şu an broadcast edilen SwarmIntent'in içeriği.
        # _on_qr_result veya özel durumlarda güncellenir.
        self._formation_type:  str   = 'OKBASI'
        self._drone_spacing:   float = 5.0
        self._target_yaw:      float = 0.0
        self._target_pos:      Point = Point()
        self._drone_altitude:  float = 10.0
        self._detach_drone_id: int   = 0    # 0 = kimse ayrılmıyor
        self._zone_color:      str   = ''
        self._maneuver_active: bool  = False
        self._maneuver_pitch:  float = 0.0
        self._maneuver_roll:   float = 0.0
        self._join_drone_id:   int   = 0    # 0 = kimse katılmıyor

        # ── YEDEK DRONE KUYRUĞU ───────────────────────────────
        # STANDBY drone'lar join_request gönderir, burada saklanır.
        # Slot açılınca (başka drone DISARM_WAIT'e düşünce)
        # en iyi aday (max batarya + ready=True) çağrılır.
        self._standby_queue: dict = {}   # drone_id → JoinRequest msg

        # ══════════════════════════════════════════════════════
        # PUBLISHER'LAR
        # ══════════════════════════════════════════════════════

        # 1) /swarm/intent — SADECE LİDER YAZAR
        #    Tüm sürünün ne yapacağını söyleyen ana komut paketi.
        self.intent_pub = self.create_publisher(
            SwarmIntent, '/swarm/intent', 10
        )

        # 2) /swarm/formation_cmd — SADECE LİDER YAZAR
        #    formation_controller için ayrı ve sade formasyon bilgisi.
        self.formation_cmd_pub = self.create_publisher(
            FormationCmd, '/swarm/formation_cmd', 10
        )

        # 3) /swarm/leader_id — TÜM DRONE'LAR YAYINLAR
        #    Hesaplanan lider ID'sini tüm sisteme duyurur.
        #    Tüm drone'lar aynı değeri yayınlamalı → tutarlılık.
        self.leader_id_pub = self.create_publisher(
            UInt8, '/swarm/leader_id', 10
        )

        # ══════════════════════════════════════════════════════
        # ABONELİKLER
        # ══════════════════════════════════════════════════════

        # 1) Tüm drone'ların state'ini dinle → HEARTBEAT MEKANİZMASI
        #    lambda + default arg (did=i) → Python closure trap'i önler!
        #    (Döngü içinde lambda: her lambda aynı i'yi yakalar → BUG)
        #    (did=i ile her lambda kendi did'ini yakalar → DOĞRU)
        self._state_subs = []
        for i in range(1, self.num_drones + 1):
            sub = self.create_subscription(
                LocalState,
                f'/drone{i}/local_state',
                lambda msg, did=i: self._on_local_state(msg, did),
                10
            )
            self._state_subs.append(sub)

        # 2) Görev başlatma komutu (GCS → mission_fsm → bu topic)
        #    MODE GATING: TASK1_ACTIVE'de GCS başka komut göndermez.
        self.create_subscription(
            TaskTrigger, '/swarm/task_trigger', self._on_task_trigger, 10
        )
        
        # 2b) QR haritası hazır sinyali (qr_perception SetQRMap sonrası)
        self.create_subscription(
            Bool, '/swarm/qr_map_ready', self._on_qr_map_ready, 10
        )

        # 3) QR kod sonucu (qr_perception → bu topic → intent güncellenir)
        #    Kamera drone'u veya failover drone'u yayınlar.
        self.create_subscription(
            QRResult, '/qr/result', self._on_qr_result, 10
        )

        # 4) Yedek drone katılma isteği
        #    STANDBY drone'lar periyodik olarak buraya yazar.
        self.create_subscription(
            JoinRequest, '/swarm/join_request', self._on_join_request, 10
        )

        # 5) /swarm/intent'i BİZ DE DİNLİYORUZ! (lider olmayan zamanlarda)
        #    SEQ TAKIBI İÇİN: Lider değişiminde yeni lider
        #    son seq'i buradan öğrenir → _global_last_seq + 1'den başlar.
        #    Kendi mesajlarımızı da alırız — bu normaldir, sorun yaratmaz.
        self.create_subscription(
            SwarmIntent, '/swarm/intent', self._on_intent_seen, 10
        )

        # ══════════════════════════════════════════════════════
        # PERİYODİK GÖREVLER (Timer)
        # ══════════════════════════════════════════════════════

        # Heartbeat kontrol + lider seçimi: 100ms
        # 600ms timeout için 6 kontrol → erken tespit
        self.create_timer(0.1, self._check_heartbeats_and_elect)

        # Intent yayını (sadece lider ise): 500ms (2 Hz)
        # Lider düşerse drone'lar en fazla 500ms pozisyon hold yapar
        self.create_timer(0.5, self._publish_intent_if_leader)

        # Lider ID yayını: 200ms (tüm drone'lar yayınlar)
        self.create_timer(0.2, self._publish_leader_id)

        self.get_logger().info(
            f'✅ [{self.ns}] intent_coordinator HAZIR!\n'
            f'   Heartbeat timeout : {self.HEARTBEAT_TIMEOUT_MS}ms\n'
            f'   Intent publish    : 2 Hz (500ms)\n'
            f'   Leader check      : 10 Hz (100ms)'
        )

    # ══════════════════════════════════════════════════════════════════════
    # CALLBACK'LER
    # ══════════════════════════════════════════════════════════════════════

    def _on_local_state(self, msg: LocalState, source_id: int):
        """
        Başka bir drone'dan gelen local_state mesajını işle.

        İKİ GÖREV:
          1) Heartbeat güncelle (drone canlı, son görülme zamanını kaydet)
          2) State değişimini tespit et → özel durumlar için aksiyon al

        SEQ FİLTRESİ:
          seq <= son_görülen → TEKRAR veya ESKİ mesaj → DROP
          Bu filtre, aynı mesajın Wi-Fi'da çoğaltılıp gelmesini engeller.

        KRİTİK DURUMLAR:
          DISARM_WAIT    → Drone indi, slot açıldı → yedek drone var mı?
          SAFETY_HOLD    → Drone devre dışı → lider yeniden seç
          PILOT_OVERRIDE → Drone RC kontrolünde → sürüden çıkar
        """
        # ── SEQ FİLTRESİ ─────────────────────────────────────────────────
        if msg.seq <= self._drone_hb_seq.get(source_id, 0):
            return  # Eski veya tekrar eden mesaj → DROP

        prev_state = self._drone_state.get(source_id, DroneState.STANDBY)

        # ── HEARTBEAT GÜNCELLE ───────────────────────────────────────────
        self._drone_state[source_id]  = msg.state
        self._drone_hb_ns[source_id]  = self.get_clock().now().nanoseconds
        self._drone_hb_seq[source_id] = msg.seq
        self._drone_seen[source_id]   = True

        # ── STATE DEĞİŞİMİ TESPİTİ ──────────────────────────────────────
        if prev_state == msg.state:
            return  # Değişim yok, işlem gerekmiyor

        self.get_logger().info(
            f'📊 [{self.ns}] drone{source_id}: '
            f'{prev_state} → {msg.state}'
        )

        # ── ÖZEL DURUM: DISARM_WAIT ───────────────────────────────────────
        # Bir drone indi! Yedek drone kuyruğu doluysa REJOIN başlat.
        # SADECE LİDER bu kararı verir (tek kaynak kuralı — §5.3).
        if msg.state == DroneState.DISARM_WAIT and self._is_leader:
            self.get_logger().info(
                f'🔔 [{self.ns}] drone{source_id} DISARM_WAIT! '
                f'Yedek drone kontrol ediliyor...'
            )
            self._trigger_rejoin_if_standby(disarmed_id=source_id)

        # ── ÖZEL DURUM: SAFETY_HOLD / PILOT_OVERRIDE ─────────────────────
        # Drone devre dışı → lider seçimi sonraki heartbeat kontrolünde yapılır.
        elif msg.state in DroneState.INACTIVE:
            self.get_logger().warn(
                f'⚠️  [{self.ns}] drone{source_id} devre dışı: {msg.state} '
                f'→ aktif listeden çıkarıldı, lider yeniden seçilecek'
            )

        # ── ÖZEL DURUM: REJOIN → FLYING ──────────────────────────────────
        # Sürüye yeniden katılan drone FLYING oldu → join tamamlandı.
        elif msg.state == DroneState.FLYING and prev_state == DroneState.REJOIN:
            self.get_logger().info(
                f'✅ [{self.ns}] drone{source_id} REJOIN → FLYING! '
                f'Sürüye katılım tamamlandı.'
            )
            # join_drone_id temizle (artık gerekli değil)
            if self._join_drone_id == source_id:
                self._join_drone_id = 0
            # Normal navigasyon moduna dön
            if self._mission_phase == MissionPhase.REJOINING:
                self._mission_phase = MissionPhase.NAVIGATING

    def _on_qr_map_ready(self, msg: Bool):
        """QR haritası yüklendi sinyali (SetQRMap başarılı olunca)."""
        if msg.data:
            self._qr_map_ready = True
            self.get_logger().info(
                f'[{self.ns}] ✅ QR HARİTASI HAZIR — TASK1 başlatılabilir'
            )
        else:
            self._qr_map_ready = False
            self.get_logger().warn(f'[{self.ns}] ⚠️ QR haritası geçersiz kılındı')

    def _on_task_trigger(self, msg: TaskTrigger):
        """
        GCS'den gelen görev başlatma / durdurma komutunu işle.

        TASK1 → Otonom sürü görevi (QR navigasyon + birey çıkarma/ekleme)
        TASK2 → Yarı otonom (joystick kontrolü — şimdilik gelecek sürümde)

        MODE GATING: TASK1_ACTIVE olunca GCS'nin diğer publisher'ları
        yazılımsal olarak kapatılır (mission_fsm bunu yönetir).
        Şartname §5.3: Görev sırasında GCS müdahalesi imkânsız.
        """
        if msg.task_type != 'TASK1':
            self.get_logger().info(
                f'[{self.ns}] task_trigger: {msg.task_type} — '
                f'TASK2 henüz desteklenmiyor, yoksayıldı'
            )
            return

        if msg.start:
            # ── QR HARİTASI KONTROL (JÜRİ KOORDİNATI) ────────────────
            if not hasattr(self, '_qr_map_ready'):
                self._qr_map_ready = False
            
            if not self._qr_map_ready:
                self.get_logger().warn(
                    f'[{self.ns}] ⚠️ QR HARİTASI HENÜZ YÜKLÜ DEĞİL! '
                    f'task_trigger yoksayıldı. Lütfen jüri koordinatlarını yükle veya '
                    f'default map ile başla.'
                )
                return
            
            if self._task_active:
                self.get_logger().warn(
                    f'[{self.ns}] Görev zaten aktif, task_trigger yoksayıldı'
                )
                return

            self.get_logger().info(
                f'\n🚀 [{self.ns}] ══════════════════════════════════\n'
                f'   GÖREV 1 BAŞLIYOR! TASK1_ACTIVE\n'
                f'   team_id: {msg.team_id}\n'
                f'   Drone\'lar ARM edilecek → sürü kalkıyor!\n'
                f'   ══════════════════════════════════'
            )
            self._task_active   = True
            self._mission_phase = MissionPhase.ARMING
            self._all_qr_done   = False
            self._current_qr_id = 1

        else:
            self.get_logger().info(
                f'⛔ [{self.ns}] Görev durduruldu (task_type={msg.task_type})'
            )
            self._task_active   = False
            self._mission_phase = MissionPhase.IDLE

    def _on_qr_result(self, msg: QRResult):
        """
        QR kod okundu — içeriği parse et, intent parametrelerini güncelle.

        SADECE LİDER İŞLER:
          Tüm drone'lar bu mesajı alır ama sadece lider intent değiştirir.
          Lider olmayan drone'lar yoksayar.

        TEKRAR ENGEL:
          qr_id <= son işlenen → DROP (aynı QR'ı iki kez işleme).

        QR İŞLEME SIRASI (Şartname §14):
          formasyon değişimi → pitch/roll manevrası →
          irtifa değişimi → birey çıkarma/ekleme → sonraki QR

        KAMERA DRONE KORUMASI (_process_detach_command içinde):
          detach_drone_id == camera_drone_id → başka drone seç!
        """
        if not self._is_leader:
            return  # Sadece lider işler

        if not self._task_active:
            return

        # ── TEKRAR ENGEL ─────────────────────────────────────────────────
        if msg.qr_id <= self._last_qr_seq:
            self.get_logger().debug(
                f'[{self.ns}] QR {msg.qr_id} zaten işlendi '
                f'(son={self._last_qr_seq}), DROP'
            )
            return

        self.get_logger().info(
            f'\n📷 [{self.ns}] ══════ QR {msg.qr_id} OKUNDU ══════\n'
            f'   Formasyon : {msg.formation_type} ({msg.drone_spacing:.1f}m)'
            f'  [{"aktif" if msg.formation_active else "pasif"}]\n'
            f'   İrtifa    : {msg.altitude:.1f}m'
            f'               [{"aktif" if msg.altitude_active else "pasif"}]\n'
            f'   Manevra   : pitch={msg.pitch_deg:.1f}° roll={msg.roll_deg:.1f}°'
            f'   [{"aktif" if msg.maneuver_active else "pasif"}]\n'
            f'   Detach    : drone{msg.detach_drone_id} → {msg.zone_color}'
            f'           [{"aktif" if msg.detach_active else "pasif"}]\n'
            f'   Sonraki QR: #{msg.next_qr_id}'
            f'                      [{"SON QR!" if msg.next_qr_id == 0 else "devam"}]\n'
            f'   ═══════════════════════════════'
        )

        self._last_qr_seq   = msg.qr_id
        self._mission_phase = MissionPhase.AT_QR

        # ── 1) FORMASYON DEĞİŞİMİ ────────────────────────────────────────
        if msg.formation_active and msg.formation_type:
            self._formation_type = msg.formation_type
            self._drone_spacing  = msg.drone_spacing
            self.get_logger().info(
                f'📐 [{self.ns}] Formasyon: {msg.formation_type}, '
                f'{msg.drone_spacing:.1f}m aralık'
            )

        # ── 2) MANEVRA (Pitch/Roll) ──────────────────────────────────────
        self._maneuver_active = msg.maneuver_active
        if msg.maneuver_active:
            self._maneuver_pitch = msg.pitch_deg
            self._maneuver_roll  = msg.roll_deg
            self.get_logger().info(
                f'🔄 [{self.ns}] Manevra: pitch={msg.pitch_deg:.1f}°, '
                f'roll={msg.roll_deg:.1f}°'
            )

        # ── 3) İRTİFA DEĞİŞİMİ ──────────────────────────────────────────
        if msg.altitude_active and msg.altitude > 0.0:
            self._drone_altitude = msg.altitude
            self.get_logger().info(
                f'📏 [{self.ns}] Yeni irtifa: {msg.altitude:.1f}m'
            )

        # ── 4) BİREY ÇIKARMA ─────────────────────────────────────────────
        if msg.detach_active:
            self._process_detach_command(msg)

        # ── 5) SONRAKI QR VEYA RETURN HOME ──────────────────────────────
        if msg.next_qr_id == 0:
            # Bu son QR! → Tüm görevler bitti → Eve dönüş
            self.get_logger().info(
                f'\n🏁 [{self.ns}] SON QR işlendi! '
                f'Eve dönüş başlıyor (RETURN_HOME)...'
            )
            self._all_qr_done   = True
            self._mission_phase = MissionPhase.RETURNING
        else:
            # Bir sonraki QR'a git
            self._current_qr_id = msg.next_qr_id
            self._target_pos    = msg.qr_position    # QRResult içindeki pozisyon
            self._target_yaw    = self._compute_yaw_to(msg.qr_position)
            self._mission_phase = MissionPhase.NAVIGATING
            self.get_logger().info(
                f'➡️  [{self.ns}] Sonraki hedef: QR#{self._current_qr_id} '
                f'@ ({msg.qr_position.x:.1f}, {msg.qr_position.y:.1f}m) '
                f'yaw={math.degrees(self._target_yaw):.1f}°'
            )

    def _on_join_request(self, msg: JoinRequest):
        """
        Yedek drone'un katılma isteğini kuyruğa ekle.

        STANDBY state'indeki drone'lar 0.5 Hz bu mesajı gönderir.
        Bir slot açıldığında (drone DISARM_WAIT'e düşünce)
        _trigger_rejoin_if_standby() en iyi adayı seçer.

        ADAYLIK KRİTERLERİ:
          ready=True    → Gerçekten hazır (sensörler kalibrasyon tamam)
          battery_pct   → Daha dolu batarya = daha iyi aday
        """
        if not msg.ready:
            return  # Hazır değil, kaydetme

        # Kendi join_request'imizi işleme (echo önlemi)
        if msg.drone_id == self.drone_id:
            return

        # Kuyruğa ekle / güncelle (aynı drone yeni mesaj gönderirse güncellenir)
        self._standby_queue[msg.drone_id] = msg
        self.get_logger().debug(
            f'[{self.ns}] Yedek drone{msg.drone_id} kuyruğa eklendi '
            f'(battery={msg.battery_pct:.0f}%, ready={msg.ready})'
        )

    def _on_intent_seen(self, msg: SwarmIntent):
        """
        Ağda yayınlanan /swarm/intent mesajını gözlemle.

        NEDEN LİDER OLMAYAN DRONE DA BU TOPIC'İ DİNLER?
        ─────────────────────────────────────────────────
        SEQ SÜREKLİLİĞİ için:
          Lider drone düşünce yeni lider seçilir.
          Yeni lider, _global_last_seq + 1'den seq başlatır.
          Bu sayede local_fsm'deki çift filtre:
            - Eski lider seq=50 gönderdi
            - Yeni lider seq=51'den başlıyor
            - local_fsm seq=51 > son_seq=50 → FİLTRE GEÇİYOR ✅
            - Eğer yeni lider seq=1'den başlasaydı →
              local_fsm seq=1 < son_seq=50 → DROP ❌ → FELAKET

        Kendi mesajlarımızı da alırız (publisher → kendi subscriber)
        Bu normaldir, seq güncelleme mantığı bu durumda da doğru çalışır.
        """
        if msg.seq > self._global_last_seq:
            self._global_last_seq = msg.seq

    # ══════════════════════════════════════════════════════════════════════
    # LİDER SEÇİMİ VE HEARTBEAT KONTROLÜ
    # ══════════════════════════════════════════════════════════════════════

    def _check_heartbeats_and_elect(self):
        """
        Her 100ms çalışır. İki görev:
          1) 600ms timeout kontrolü → timed-out drone'ları SAFETY_HOLD say
          2) Lider seçimi → FLYING drone'lar arasından min(id)

        LİDER TRANSITION TESPİTİ:
          _was_leader → _is_leader geçişi izlenir.
          Yeni lider oldum → _my_intent_seq = _global_last_seq + 1
          Böylece local_fsm'deki filtre doğru çalışır.
        """
        now_ns    = self.get_clock().now().nanoseconds
        timeout_ns = int(self.HEARTBEAT_TIMEOUT_MS * 1_000_000)

        # ── TIMEOUT KONTROLÜ ─────────────────────────────────────────────
        for did in range(1, self.num_drones + 1):
            if did == self.drone_id:
                continue  # Kendimizi timeout etmiyoruz (aynı makinede)

            if not self._drone_seen[did]:
                continue  # Henüz hiç mesaj gelmedi, timeout sayma

            # Devre dışı state'teyse zaten inactive sayılıyor
            if self._drone_state[did] in DroneState.INACTIVE:
                continue

            elapsed_ns = now_ns - self._drone_hb_ns[did]
            if elapsed_ns > timeout_ns:
                old_state = self._drone_state[did]
                self._drone_state[did] = DroneState.SAFETY_HOLD
                self.get_logger().warn(
                    f'⏱️  [{self.ns}] TIMEOUT! drone{did} '
                    f'{elapsed_ns / 1e6:.0f}ms yanıt vermedi. '
                    f'{old_state} → SAFETY_HOLD (varsayım)'
                )

        # ── LİDER SEÇİMİ ─────────────────────────────────────────────────
        self._was_leader = self._is_leader
        new_leader = self._elect_leader()

        if new_leader != self._leader_id:
            old_leader      = self._leader_id
            self._leader_id = new_leader
            self._is_leader = (new_leader == self.drone_id)

            if new_leader > 0:
                self.get_logger().info(
                    f'👑 [{self.ns}] Yeni lider: drone{new_leader} '
                    f'(eski: drone{old_leader if old_leader > 0 else "?"})'
                )
            else:
                self.get_logger().warn(
                    f'⚠️  [{self.ns}] LİDER YOK! Tüm drone\'lar pozisyon hold...'
                )
        else:
            self._is_leader = (self._leader_id == self.drone_id)

        # ── YENİ LİDER OLDUM MU? ─────────────────────────────────────────
        if self._is_leader and not self._was_leader:
            # Az önce lider seçildim!
            # SEQ SÜREKLİLİĞİ: Eski liderin son seq + 1'den başla.
            # local_fsm'deki çift filtre: yeni seq > eski seq → DROP olmaz.
            self._my_intent_seq = self._global_last_seq + 1
            self.get_logger().info(
                f'\n👑 [{self.ns}] ═══════════════════════════════\n'
                f'   BEN LİDER OLDUM!\n'
                f'   Global son seq : {self._global_last_seq}\n'
                f'   Benim başlangıç: {self._my_intent_seq}\n'
                f'   ═══════════════════════════════'
            )

    def _elect_leader(self) -> int:
        """
        Bully Algoritması (Basitleştirilmiş):
        Canlı FLYING/REJOIN drone'lar arasından min(id) lider.

        ARMING FAZINDA (Henüz kimse FLYING değil):
          STANDBY/IDLE drone'lar arasından min(id) seçilir.
          Bu sayede kalkış öncesi de bir lider var → IDLE intent gönderilir.

        TUTARLILIK GARANTİSİ:
          Tüm drone'lar aynı _drone_state ve _drone_seen verilerini görünce
          aynı hesaplamayı yapar → AYNI lider sonucu → Merkezi seçime gerek yok!

        DÖNÜŞ: lider drone_id (0 = lider yok)
        """
        if not self._task_active:
            return 0  # Görev başlamadı → lider seçme

        # ── NORMAL DURUM: FLYING/REJOIN olanlar ──────────────────────────
        eligible = [
            did for did in range(1, self.num_drones + 1)
            if self._drone_state[did] in DroneState.LEADER_ELIGIBLE
            and self._is_alive(did)
        ]
        if eligible:
            return min(eligible)

        # ── ARMING FAZINDA: Henüz kimse FLYING değil ─────────────────────
        # task_trigger geldi, drone'lar ARM edilmekte
        # STANDBY/IDLE olanlar arasından seç
        pre_flight = [
            did for did in range(1, self.num_drones + 1)
            if self._drone_state[did] in {DroneState.STANDBY, DroneState.IDLE}
            and self._is_alive(did)
        ]
        if pre_flight:
            return min(pre_flight)

        return 0  # Hiç uygun aday yok → lider yok

    def _is_alive(self, drone_id: int) -> bool:
        """
        Drone canlı mı? Son 600ms içinde heartbeat geldi mi?

        KENDİ DRONE'UMUZ:
          Aynı RPi4'te çalıştığımız için kendi heartbeat'imizi
          direkt göremeyiz (loopback). Ama local_fsm ile aynı
          process'teyiz → her zaman canlı say.

        HİÇ MESAJ GELMEDİ:
          _drone_seen=False → "henüz görülmedi" → canlı sayma.
          (timeout değil, sadece henüz veri yok)
        """
        if drone_id == self.drone_id:
            return True  # Kendi drone'umuz: aynı makinede → her zaman canlı

        if not self._drone_seen[drone_id]:
            return False  # Henüz hiç heartbeat gelmedi

        elapsed_ns = self.get_clock().now().nanoseconds - self._drone_hb_ns[drone_id]
        return elapsed_ns < int(self.HEARTBEAT_TIMEOUT_MS * 1_000_000)

    # ══════════════════════════════════════════════════════════════════════
    # INTENT ÜRETME VE YAYINLAMA
    # ══════════════════════════════════════════════════════════════════════

    def _publish_intent_if_leader(self):
        """
        Lider olduğumuzda periyodik intent yayınla (2 Hz, 500ms).

        YAYINLANMAZ EĞER:
          - Görev aktif değilse (task_active=False)
          - Lider değilsek

        2 HZ NEDENİ:
          Lider düşerse drone'lar en fazla 500ms pozisyon hold yapar.
          Daha yüksek frekans gereksiz Wi-Fi trafiği oluşturur.
          3 drone × 2 Hz = 6 mesaj/sn → kabul edilebilir.
        """
        if not self._is_leader or not self._task_active:
            return

        intent = self._build_intent()
        if intent is None:
            return

        self.intent_pub.publish(intent)

        # Formasyon komutunu ayrıca yayınla (formation_controller için)
        self._publish_formation_cmd()

    def _build_intent(self):
        """
        Mevcut görev fazına göre SwarmIntent mesajı oluştur.

        Her çağrıda _my_intent_seq bir artar.
        Bu monotonic artış, local_fsm'in çift filtresinin temeldir:
          seq 10 geldi, sonra seq 11 geldi → 11 > 10 → DROP olmaz ✅
          seq 10 geldi, sonra seq 8 geldi  → 8 < 10 → DROP ✅ (eski lider)

        GÖREV FAZ → task_id BAĞLAMASI:
          ARMING     → 'IDLE'          (drone'lar ARM edilsin)
          NAVIGATING → 'QR_NAVIGATE'   (hedefe git)
          AT_QR      → 'QR_NAVIGATE' veya 'MANEUVER'
          DETACHING  → 'DETACH'        (belirtilen drone ayrılsın)
          REJOINING  → 'REJOIN'        (belirtilen drone katılsın)
          RETURNING  → 'RETURN_HOME'   (eve dön)
        """
        self._my_intent_seq += 1

        msg             = SwarmIntent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.seq         = self._my_intent_seq
        msg.leader_id   = self.drone_id

        # ── GÖREV FAZ → task_id ──────────────────────────────────────────
        phase = self._mission_phase
        if phase == MissionPhase.ARMING:
            msg.task_id = 'IDLE'
        elif phase in (MissionPhase.NAVIGATING, MissionPhase.AT_QR):
            msg.task_id = 'MANEUVER' if self._maneuver_active else 'QR_NAVIGATE'
        elif phase == MissionPhase.DETACHING:
            msg.task_id = 'DETACH'
        elif phase == MissionPhase.REJOINING:
            msg.task_id = 'REJOIN'
        elif phase in (MissionPhase.RETURNING, MissionPhase.COMPLETE):
            msg.task_id = 'RETURN_HOME'
        else:
            msg.task_id = 'IDLE'

        # ── ORTAK INTENT ALANLARI ────────────────────────────────────────
        msg.formation_type    = self._formation_type
        msg.drone_spacing     = self._drone_spacing
        msg.target_yaw        = self._target_yaw
        msg.target_pos        = self._target_pos
        msg.drone_altitude    = self._drone_altitude
        msg.detach_drone_id   = self._detach_drone_id
        msg.zone_color        = self._zone_color
        msg.maneuver_active   = self._maneuver_active
        msg.maneuver_pitch_deg = self._maneuver_pitch
        msg.maneuver_roll_deg  = self._maneuver_roll
        msg.join_drone_id     = self._join_drone_id
        msg.qr_seq            = self._last_qr_seq
        msg.active_drone_count = self._get_active_drone_count()

        return msg

    def _publish_formation_cmd(self):
        """
        formation_controller için ayrı FormationCmd yayınla.

        SwarmIntent içinde formasyon bilgisi zaten var.
        Ama formation_controller için sade bir arayüz daha temiz.
        (İleride formasyon controller'ı SwarmIntent okumak istemeyebilir)
        """
        msg = FormationCmd()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.formation_type  = self._formation_type
        msg.drone_spacing   = self._drone_spacing
        msg.target_yaw      = self._target_yaw
        msg.active_drone_count = self._get_active_drone_count()
        self.formation_cmd_pub.publish(msg)

    def _publish_leader_id(self):
        """
        Şu anki lider ID'sini yayınla (5 Hz, 200ms).
        Tüm drone'lar bunu yayınlar — tutarlılık için.
        local_fsm /swarm/leader_id'yi okuyabilir.
        """
        msg      = UInt8()
        msg.data = self._leader_id
        self.leader_id_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════════
    # GÖREV AKIŞ YÖNETİMİ
    # ══════════════════════════════════════════════════════════════════════

    def _process_detach_command(self, qr: QRResult):
        """
        QR'dan gelen birey çıkarma komutunu işle.

        KAMERA DRONE KORUMASI (Kritik!):
        ─────────────────────────────────
        Şartname: QR algılamanın sürdürülebilmesi için en az
        bir İHA'da kamera olmalıdır. Kamera drone'u detach edilirse
        QR algılama kesilir → görev başarısız!

        Eğer QR'da kamera drone'u yazıyorsa → başka FLYING drone seç.
        Eğer başka drone yoksa → detach komutu yoksay (hata logla).

        ZONE COLOR:
        ───────────
        Renk zone koordinatı precision_landing'de kullanılacak.
        intent_coordinator zone koordinatını bilmeyebilir (qr_perception verir).
        Şimdilik zone_color string olarak taşınıyor, precision_landing halledecek.
        """
        detach_id = qr.detach_drone_id

        # ── KAMERA DRONE KORUMASI ─────────────────────────────────────────
        if detach_id == self.camera_drone_id:
            # Kamera drone seçilmiş! Alternatif bul.
            alternatives = [
                did for did in range(1, self.num_drones + 1)
                if self._drone_state[did] == DroneState.FLYING
                and did != self.camera_drone_id
                and self._is_alive(did)
            ]
            if alternatives:
                old_id    = detach_id
                detach_id = min(alternatives)  # En küçük ID'li alternatif
                self.get_logger().warn(
                    f'🛡️  [{self.ns}] KAMERA KORUMASI!\n'
                    f'   QR\'da drone{old_id} (kamera) yazıyor ama koruma aktif.\n'
                    f'   Alternatif: drone{detach_id} detach ediliyor.'
                )
            else:
                self.get_logger().error(
                    f'❌ [{self.ns}] DETACH HATASI!\n'
                    f'   Kamera drone koruması aktif ama alternatif FLYING drone yok!\n'
                    f'   Detach komutu yoksayılıyor.'
                )
                return

        self._detach_drone_id = detach_id
        self._zone_color      = qr.zone_color
        self._mission_phase   = MissionPhase.DETACHING

        self.get_logger().info(
            f'✂️  [{self.ns}] DETACH: drone{detach_id} → '
            f'{qr.zone_color} zone\'a iniyor'
        )

    def _trigger_rejoin_if_standby(self, disarmed_id: int):
        """
        Bir drone DISARM_WAIT'e düştü → Yedek drone varsa REJOIN başlat.

        SADECE LİDER ÇAĞIRIR (tek kaynak kuralı).

        YEDEK DRONE SEÇİMİ:
          Kuyruktan max batarya + ready=True olan seçilir.
          Batarya dolu olan daha uzun süre sürüde kalabilir.

        REJOIN SONRASI:
          join_drone_id dolu tutulur, local_fsm'ler bunu okur.
          İlgili drone'un local_fsm'i DISARM_WAIT → REARM → REJOIN → FLYING geçişini yapar.
        """
        best = self._get_best_standby_drone()
        if best == 0:
            self.get_logger().info(
                f'[{self.ns}] Yedek drone kuyruğu boş → '
                f'drone{disarmed_id}\'nin yerine kimse katılmıyor'
            )
            self._join_drone_id = 0
            return

        self._join_drone_id = best
        self._mission_phase = MissionPhase.REJOINING

        # Standby kuyruğundan çıkar (artık aktif süreçte)
        self._standby_queue.pop(best, None)

        self.get_logger().info(
            f'🔄 [{self.ns}] REJOIN: drone{best} sürüye katılıyor '
            f'(drone{disarmed_id}\'nin yerine, '
            f'battery={self._standby_queue.get(best, type("", (), {"battery_pct": 0})).battery_pct:.0f}%)'
        )

    # ══════════════════════════════════════════════════════════════════════
    # YARDIMCI FONKSİYONLAR
    # ══════════════════════════════════════════════════════════════════════

    def _get_active_drone_count(self) -> int:
        """
        Şu an havada olan (aktif) drone sayısını döndür.

        formation_controller fallback için:
          active_drone_count < 3 → 2'li LINE formasyonuna geç
          (3'lü formasyon = ok başı / V / çizgi; 2'li = sadece çizgi)
        """
        return sum(
            1 for did in range(1, self.num_drones + 1)
            if self._drone_state[did] in DroneState.AIRBORNE
            and self._is_alive(did)
        )

    def _get_best_standby_drone(self) -> int:
        """
        Yedek kuyruğundan en iyi adayı seç.

        KRİTER: ready=True + canlı + max batarya yüzdesi
        DÖNÜŞ: drone_id (0 = uygun aday yok)
        """
        candidates = [
            (msg.battery_pct, did)
            for did, msg in self._standby_queue.items()
            if msg.ready and self._is_alive(did)
        ]
        if not candidates:
            return 0
        # En yüksek batarya → en iyi aday
        return max(candidates, key=lambda x: x[0])[1]

    def _compute_yaw_to(self, target: Point) -> float:
        """
        Hedef noktaya olan yaw açısını hesapla (radyan, ENU koordinat sistemi).

        ENU'da açı:
          0 rad   = Doğu (+X)
          π/2 rad = Kuzey (+Y)
          Matematiksel pozitif = saat yönünün tersi (CCW)

        formation_controller bu açıyla formasyon offset vektörlerini döndürür.
        Böylece formasyonun "ön tarafı" her zaman hedefe bakar.

        TODO (Gelecek sürüm):
          Kendi konumumuzu /drone{id}/pose'dan alıp gerçek yaw hesapla.
          Şimdilik hedefin mutlak yönünü kullanıyoruz (yeterince doğru).
        """
        dx = target.x
        dy = target.y

        if abs(dx) < 0.01 and abs(dy) < 0.01:
            return self._target_yaw  # Hedef çok yakın, mevcut yaw'ı koru

        return math.atan2(dy, dx)   # ENU'da kuzeyden saat yönünün tersi


# ══════════════════════════════════════════════════════════════════════════════
# ANA FONKSİYON
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    """
    Node başlatma.

    TEST ETME:
    ----------
    1) Simülasyon çalışıyorken (tüm node'lar açıkken):
       ros2 run my_swarm_pkg intent_coordinator --ros-args -p drone_id:=1

    2) Lider kim? izle:
       ros2 topic echo /swarm/leader_id

    3) Intent akışını izle:
       ros2 topic echo /swarm/intent

    4) Görevi başlat (mission_fsm yokken manuel):
       ros2 topic pub /swarm/task_trigger swarm_msgs/msg/TaskTrigger \
         "{task_type: 'TASK1', start: true, team_id: 'TEST'}" --once

    5) Mock QR sonucu gönder (qr_perception yokken):
       ros2 topic pub /qr/result swarm_msgs/msg/QRResult "{
         qr_id: 1,
         formation_active: true, formation_type: 'OKBASI', drone_spacing: 5.0,
         altitude_active: true, altitude: 10.0,
         maneuver_active: false,
         detach_active: false,
         next_qr_id: 2,
         qr_position: {x: 30.0, y: 50.0, z: 0.0}
       }" --once

    6) Son QR gönder (eve dönüşü tetikle):
       ros2 topic pub /qr/result swarm_msgs/msg/QRResult "{
         qr_id: 2, next_qr_id: 0
       }" --once

    7) Tüm local_state'leri izle (ayrı terminalde):
       ros2 topic echo /drone1/local_state &
       ros2 topic echo /drone2/local_state &
       ros2 topic echo /drone3/local_state &
    """
    rclpy.init(args=args)
    node = IntentCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⛔ intent_coordinator durduruluyor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
