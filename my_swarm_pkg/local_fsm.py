#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                            local_fsm.py                                      ║
║                  Yerel Durum Makinesi — Her İHA'nın Beyni                    ║
╚══════════════════════════════════════════════════════════════════════════════╝

GENEL AÇIKLAMA:
---------------
Bu node, her drone'un "şu an ne yapıyor?" sorusunun cevabını yönetir.
Tüm durum geçişlerini (state transition) tek elden kontrol eder.

KRİTİK KURAL: /drone_{id}/cmd_mode topic'ine SADECE bu node yazar.
Başka hiçbir node bu topic'e yazamaz. Bu kural sistem güvenliğinin temelidir.
Pixhawk'a uçuş modu komutu sadece local_fsm üzerinden gider.

STATE (DURUM) MAKİNESİ:
-----------------------
                          pilot_override=True
        ┌────────────────────────────────────────────────────────┐
        │                                                        ▼
    STANDBY ──arm+intent──► IDLE ──takeoff──► FLYING ──────► PILOT_OVERRIDE
        ▲                                        │               │
        │                                        │ safety/event  │ pilot_override=False
        │                            ┌───────────┘               │
        │                            ▼                           ▼
        │                       SAFETY_HOLD ◄──────────────── FLYING
        │                            │
        │                            │ (detach_drone_id==kendi_id)
        │                            ▼
        │                         DETACH
        │                            │
        │                            ▼
        │                        LAND_ZONE (precision_landing aktif)
        │                            │
        │                            ▼
        │                       DISARM_WAIT
        │                            │
        │               ┌────────────┴──────────────┐
        │               │ (join_drone_id==kendi_id)  │ (son görev bitti)
        │               ▼                            ▼
        └──────────── REARM                     RETURN_HOME
                        │                            │
                        ▼                            ▼
                      REJOIN                      LANDING
                        │
                        ▼
                      FLYING

ÇİFT FİLTRE SİSTEMİ:
--------------------
SwarmIntent mesajları geldiğinde iki kontrol yapılır:
  1) msg.seq < self._last_intent_seq → ESKİ LİDERDEN KALAN PAKET → DROP
  2) msg.seq == self._last_intent_seq VE msg.header.stamp < son_stamp → DROP
Bu sayede lider değişiminde eski liderin geç gelen paketleri uygulanmaz.

ŞARTNAME KARŞILAŞTIRMASI:
  §5.3  Dağıtık mimari  → Her drone kendi state machine'ini çalıştırır ✅
  §5.5.4 Kill-switch    → PILOT_OVERRIDE state'i ile yönetilir ✅
  §5.5   Failsafe       → SAFETY_HOLD state'i GCS'ye sormadan karar verir ✅
  §15   Birey çıkarma  → DETACH → LAND_ZONE → DISARM_WAIT akışı ✅
  §15   Birey ekleme   → REARM → REJOIN → FLYING akışı ✅

ÇALIŞTIRMA:
-----------
  ros2 run my_swarm_pkg local_fsm --ros-args -p drone_id:=1
  ros2 run my_swarm_pkg local_fsm --ros-args -p drone_id:=2
  ros2 run my_swarm_pkg local_fsm --ros-args -p drone_id:=3
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.time import Time

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

from swarm_msgs.msg import LocalState, SwarmIntent, JoinRequest, SafetyEvent


# ══════════════════════════════════════════════════════════════════════════════
# STATE SABİTLERİ
# Tüm geçerli state değerleri burada tanımlı. Magic string kullanmıyoruz.
# ══════════════════════════════════════════════════════════════════════════════
class State:
    STANDBY         = 'STANDBY'
    IDLE            = 'IDLE'
    FLYING          = 'FLYING'
    DETACH          = 'DETACH'
    LAND_ZONE       = 'LAND_ZONE'
    DISARM_WAIT     = 'DISARM_WAIT'
    REARM           = 'REARM'
    REJOIN          = 'REJOIN'
    RETURN_HOME     = 'RETURN_HOME'
    LANDING         = 'LANDING'
    SAFETY_HOLD     = 'SAFETY_HOLD'
    PILOT_OVERRIDE  = 'PILOT_OVERRIDE'


# ══════════════════════════════════════════════════════════════════════════════
# GÖREV SABİTLERİ
# SwarmIntent.task_id'de kullanılan değerler
# ══════════════════════════════════════════════════════════════════════════════
class Task:
    IDLE         = 'IDLE'
    QR_NAVIGATE  = 'QR_NAVIGATE'
    DETACH       = 'DETACH'
    REJOIN       = 'REJOIN'
    RETURN_HOME  = 'RETURN_HOME'
    MANEUVER     = 'MANEUVER'


class LocalFSM(Node):
    """
    Her drone için çalışan yerel durum makinesi.

    Temel sorumluluklar:
      1) SwarmIntent mesajlarını çift filtreden geçirerek işle
      2) State geçişlerini yönet
      3) drone_interface'e cmd_mode ve cmd_arm komutlarını gönder
      4) intent_coordinator'a local_state yayınla
      5) STANDBY iken join_request gönder
      6) Pilot override ve güvenlik olaylarına tepki ver
    """

    def __init__(self):
        super().__init__('local_fsm')

        # ══════════════════════════════════════════════════════
        # PARAMETRE OKUMA
        # ══════════════════════════════════════════════════════
        self.declare_parameter('drone_id', 1)
        self.drone_id = self.get_parameter('drone_id').value
        self.ns = f'drone{self.drone_id}'

        self.get_logger().info(
            f'🧠 local_fsm başlatılıyor...\n'
            f'   Drone ID : {self.drone_id}\n'
            f'   Topic ns : /{self.ns}/'
        )

        # ══════════════════════════════════════════════════════
        # DURUM DEĞİŞKENLERİ
        # ══════════════════════════════════════════════════════

        # Şu anki state
        self._state: str = State.STANDBY

        # State yayın sıra numarası (her state publish'te artar)
        self._state_seq: int = 0

        # ── ÇİFT FİLTRE DEĞİŞKENLERİ ────────────────────────
        # Son işlenen SwarmIntent mesajının seq numarası
        # Yeni gelen seq < bu değerse → ESKİ MESAJ → DROP
        self._last_intent_seq: int = 0

        # Son işlenen SwarmIntent mesajının timestamp'i
        # Aynı seq ama daha eski timestamp → DROP
        self._last_intent_stamp: Time = Time()

        # ── PİLOT OVERRIDE TAKİBİ ────────────────────────────
        # drone_interface'ten gelen son pilot_override değeri
        self._pilot_override: bool = False

        # Pilot override önceki state'i sakla (override bitince dön)
        self._pre_override_state: str = State.STANDBY

        # ── GÖREV TAKİBİ ──────────────────────────────────────
        # Son işlenen SwarmIntent'ten gelen görev bilgileri
        self._current_task: str = Task.IDLE
        self._last_qr_seq: int = 0       # Aynı QR'ı iki kez işlemeyi engeller

        # Detach görevi için: biz mi ayrılıyoruz?
        self._pending_detach: bool = False
        self._pending_join: bool = False   # Biz mi sürüye katılıyoruz?

        # DISARM_WAIT → REARM için: beklemede miyiz?
        self._waiting_for_join_signal: bool = False

        # ── KALKIŞ YÜKSEKLIĞI ─────────────────────────────────
        # İlk kalkışta ve REJOIN'da hedef irtifa
        self._target_altitude: float = 5.0   # metre (intent'ten güncellenir)

        # ══════════════════════════════════════════════════════
        # PUBLISHER'LAR
        # ══════════════════════════════════════════════════════

        # 1) State yayınla → intent_coordinator ve swarm_comm okur
        self.local_state_pub = self.create_publisher(
            LocalState, f'/{self.ns}/local_state', 10
        )

        # 2) Mod komutu → drone_interface okur → Pixhawk'a iletir
        # SADECE bu node yazar! (sistem güvenlik kuralı)
        self.cmd_mode_pub = self.create_publisher(
            String, f'/{self.ns}/cmd_mode', 10
        )

        # 3) ARM/DISARM komutu → drone_interface okur → Pixhawk'a iletir
        self.cmd_arm_pub = self.create_publisher(
            Bool, f'/{self.ns}/cmd_arm', 10
        )

        # 4) Sürüye katılma isteği (STANDBY state'inde periyodik)
        # → intent_coordinator tüm İHA'larda dinler
        self.join_request_pub = self.create_publisher(
            JoinRequest, '/swarm/join_request', 10
        )

        # ══════════════════════════════════════════════════════
        # ABONELİKLER
        # ══════════════════════════════════════════════════════

        # 1) Sürü niyeti → tüm state geçişlerinin tetikleyicisi
        #    Çift filtreden geçirilir!
        self.create_subscription(
            SwarmIntent,
            '/swarm/intent',
            self._intent_cb,
            10
        )

        # 2) Güvenlik olayı → SAFETY_HOLD geçişini tetikler
        #    GCS'ye sormadan, ANINDA tepki verilir!
        self.create_subscription(
            SafetyEvent,
            '/safety/event',
            self._safety_cb,
            10
        )

        # 3) Pilot override → PILOT_OVERRIDE state geçişi
        #    drone_interface'ten gelir (RC kill-switch / GUIDED dışı mod)
        self.create_subscription(
            Bool,
            f'/{self.ns}/pilot_override',
            self._pilot_override_cb,
            10
        )

        # ══════════════════════════════════════════════════════
        # PERİYODİK GÖREVLER (Timer)
        # ══════════════════════════════════════════════════════

        # State'i periyodik olarak yayınla (heartbeat, 10 Hz)
        # ÖNEMLİ: intent_coordinator 500ms timeout kullanıyor.
        # Biz 0.1s (10 Hz) yayın yapıyoruz → timeout içinde 5 paket gönderilir.
        # Wi-Fi jitter (10-20ms) olsa bile 4-5 paket kaybolması gerekir
        # drone'un "düştü" sayılması için. Bu FALSE-POSITIVE alarm riskini
        # pratik sıfıra indirir.
        # NEDEN 0.5s YETERSİZ?
        #   0.5s timer + 500ms timeout = tampon SIFIR
        #   Ağda tek bir 10ms gecikme → lider drone'u sürüden çıkarır!
        #   3 drone = anlamına gelir → yarışmada felaket.
        self.create_timer(0.1, self._publish_state)

        # STANDBY iken join_request gönder (2 Hz)
        # Çok sık göndermeye gerek yok, 2 Hz yeterli
        self.create_timer(0.5, self._publish_join_request_if_standby)

        self.get_logger().info(
            f'✅ [{self.ns}] local_fsm HAZIR! İlk durum: {self._state}'
        )

    # ══════════════════════════════════════════════════════════════════════
    # CALLBACK'LER
    # ══════════════════════════════════════════════════════════════════════

    def _intent_cb(self, msg: SwarmIntent):
        """
        Lider İHA'dan gelen SwarmIntent mesajını işle.

        ÇİFT FİLTRE MANTIĞI:
        ─────────────────────
        Lider değişiminde (örn: lider drone düştü, yeni lider seçildi)
        eski liderin geç gelen paketleri sisteme zarar verebilir.
        İki aşamalı filtre bunu önler:

        Filtre 1 - Seq kontrolü:
          Yeni lider seq'i seq+1'den başlatır. Eski liderin seq'leri artık
          küçük kalır ve DROP edilir.

        Filtre 2 - Timestamp kontrolü:
          Aynı seq ama daha eski timestamp → ağda sıra dışı gelen paket → DROP.

        SAFETY_HOLD ve PILOT_OVERRIDE halinde intent işleme:
          Bu state'lerde drone bağımsız hareket ettiğinden intent'leri
          yoksayıyoruz. Güvenlik önceliklidir!
        """
        # ── GÜVENLİK STATE KONTROLÜ ──────────────────────────────────────
        if self._state in (State.SAFETY_HOLD, State.PILOT_OVERRIDE):
            # Bu state'lerde bağımsız hareket ediyoruz, sürü niyeti yoksay
            return

        # ── FİLTRE 1: SEQ KONTROLÜ ───────────────────────────────────────
        if msg.seq < self._last_intent_seq:
            self.get_logger().debug(
                f'[{self.ns}] SwarmIntent DROP (eski seq): '
                f'gelen={msg.seq} < son={self._last_intent_seq}'
            )
            return

        # ── FİLTRE 2: TIMESTAMP KONTROLÜ (aynı seq) ──────────────────────
        if msg.seq == self._last_intent_seq:
            gelen_stamp = Time.from_msg(msg.header.stamp)
            if gelen_stamp < self._last_intent_stamp:
                self.get_logger().debug(
                    f'[{self.ns}] SwarmIntent DROP (eski timestamp, aynı seq={msg.seq})'
                )
                return

        # ── FİLTRE GEÇTİ: GÜNCELLE ───────────────────────────────────────
        self._last_intent_seq = msg.seq
        self._last_intent_stamp = Time.from_msg(msg.header.stamp)
        self._target_altitude = msg.drone_altitude if msg.drone_altitude > 0 else self._target_altitude
        self._current_task = msg.task_id

        # ── GÖREV DAĞILIMI ────────────────────────────────────────────────
        if msg.task_id == Task.IDLE:
            self._handle_idle_intent(msg)

        elif msg.task_id == Task.QR_NAVIGATE:
            self._handle_navigate_intent(msg)

        elif msg.task_id == Task.DETACH:
            self._handle_detach_intent(msg)

        elif msg.task_id == Task.REJOIN:
            self._handle_rejoin_intent(msg)

        elif msg.task_id == Task.RETURN_HOME:
            self._handle_return_home_intent(msg)

        elif msg.task_id == Task.MANEUVER:
            # Manevra sırasında state değişmez, formation_controller halleder
            # Ama hedef irtifayı güncelle
            self.get_logger().info(
                f'[{self.ns}] Manevra komutu alındı '
                f'(pitch={msg.maneuver_pitch_deg:.1f}° '
                f'roll={msg.maneuver_roll_deg:.1f}°)'
            )

    def _safety_cb(self, msg: SafetyEvent):
        """
        Güvenlik olayı geldi → SAFETY_HOLD state'ine geç.

        ÖNEMLİ: Bu geçiş GCS'ye SORMADAN yapılır!
        §5.5.4: Her İHA bağımsız failsafe kararı alabilmelidir.

        SAFETY_HOLD'da ne olur:
          - Pixhawk RTL moduna alınır (eve dön)
          - intent mesajları yoksayılır
          - local_state=SAFETY_HOLD yayınlanır → intent_coordinator
            bu drone'u FLYING listesinden çıkarır
        """
        # Sadece kendi drone'umuzu etkileyen olaylara bak
        # (safety_monitor drone_id'yi doldurur)
        if msg.drone_id != self.drone_id and msg.drone_id != 0:
            # drone_id=0 → broadcast (tüm drone'ları etkiler)
            return

        self.get_logger().warn(
            f'🚨 [{self.ns}] GÜVENLİK OLAYI! Tip: {msg.event_type} '
            f'| Açıklama: {msg.description}\n'
            f'   SAFETY_HOLD state\'ine geçiliyor...'
        )

        # Zaten safety hold'daysa tekrar geçme
        if self._state == State.SAFETY_HOLD:
            return

        self._transition_to(State.SAFETY_HOLD)

        # Pixhawk'a RTL komutu gönder (eve dön, güvenli iniş)
        self._send_mode('RTL')

    def _pilot_override_cb(self, msg: Bool):
        """
        drone_interface'ten gelen pilot override durumunu işle.

        True  → RC kill-switch veya GUIDED dışı mod tespit edildi
                 PILOT_OVERRIDE state'ine geç
        False → Override bitti, önceki state'e dön
                 (Eğer dönülecek state güvenli ise)
        """
        prev = self._pilot_override
        self._pilot_override = msg.data

        if msg.data and not prev:
            # Override BAŞLADI
            self.get_logger().warn(
                f'⚠️  [{self.ns}] PILOT OVERRIDE başladı! '
                f'Önceki state={self._state} saklandı.'
            )
            # Mevcut state'i sakla (override bitince buraya dönelim)
            if self._state not in (State.PILOT_OVERRIDE, State.SAFETY_HOLD):
                self._pre_override_state = self._state
            self._transition_to(State.PILOT_OVERRIDE)

        elif not msg.data and prev:
            # Override BİTTİ
            if self._state == State.PILOT_OVERRIDE:
                self.get_logger().info(
                    f'✅ [{self.ns}] PILOT OVERRIDE bitti! '
                    f'Önceki state\'e dönülüyor: {self._pre_override_state}'
                )
                # Güvenli state'lere dön
                # DETACH, LAND_ZONE gibi kritik state'lere de dönebiliriz —
                # pilot override sırasında o bölgeden uçmuş olabiliriz,
                # drone_interface şimdi tekrar GUIDED'a geçecek
                self._transition_to(self._pre_override_state)

                # GUIDED moda geri al
                if self._pre_override_state in (State.FLYING, State.DETACH,
                                                 State.LAND_ZONE, State.REJOIN,
                                                 State.RETURN_HOME):
                    self._send_mode('GUIDED')

    # ══════════════════════════════════════════════════════════════════════
    # GÖREV İŞLEYİCİLERİ (Intent Handlers)
    # ══════════════════════════════════════════════════════════════════════

    def _handle_idle_intent(self, msg: SwarmIntent):
        """
        task_id='IDLE': Görev henüz başlamadı.

        STANDBY → IDLE geçişi: intent_coordinator ilk görev öncesi
        tüm drone'lara IDLE intent gönderir. Drone'lar ARM edilir.
        """
        if self._state == State.STANDBY:
            self.get_logger().info(
                f'🚀 [{self.ns}] IDLE intent alındı, STANDBY → IDLE geçişi'
            )
            self._transition_to(State.IDLE)
            # Drone'u ARM et (motorlar hazır)
            self._send_arm(True)
            # GUIDED moda al (otonom uçuşa hazır)
            self._send_mode('GUIDED')

    def _handle_navigate_intent(self, msg: SwarmIntent):
        """
        task_id='QR_NAVIGATE': Sürü QR noktasına doğru ilerliyor.

        Olası geçişler:
          IDLE    → FLYING : İlk kalkış
          FLYING  → FLYING : Devam ediyor (zaten uçuyor)
          REJOIN  → FLYING : Sürüye katıldıktan sonra
        """
        if self._state == State.IDLE:
            self.get_logger().info(
                f'🛫 [{self.ns}] KALKIŞ! Hedef irtifa: {msg.drone_altitude:.1f}m'
            )
            self._transition_to(State.FLYING)
            self._send_mode('GUIDED')

        elif self._state == State.REJOIN:
            # Sürüye yeniden katıldık, artık FLYING sayılıyoruz
            self.get_logger().info(
                f'🤝 [{self.ns}] REJOIN → FLYING! Sürüye katıldım.'
            )
            self._transition_to(State.FLYING)

        elif self._state == State.FLYING:
            # Zaten uçuyoruz, sadece formasyon/hedef değişti
            # Yeni QR geldi mi?
            if msg.qr_seq > self._last_qr_seq:
                self._last_qr_seq = msg.qr_seq
                self.get_logger().info(
                    f'📍 [{self.ns}] Yeni QR hedefi: '
                    f'({msg.target_pos.x:.1f}, {msg.target_pos.y:.1f}, '
                    f'{msg.target_pos.z:.1f}m) | QR seq={msg.qr_seq}'
                )

    def _handle_detach_intent(self, msg: SwarmIntent):
        """
        task_id='DETACH': Birey çıkarma görevi.
        Şartname §15: Belirtilen drone ayrılıp renk bölgesine iner.

        Sadece detach_drone_id bizim ID'mize eşitse hareket ederiz.
        Diğer drone'lar bu intent'e tepki vermez (sürüde kalmaya devam eder).
        """
        if msg.detach_drone_id != self.drone_id:
            # Bu detach emri bize değil
            return

        if self._state not in (State.FLYING,):
            self.get_logger().warn(
                f'⚠️  [{self.ns}] DETACH komutu geldi ama durum={self._state}, '
                f'yoksayıldı'
            )
            return

        self.get_logger().info(
            f'🎯 [{self.ns}] DETACH! Hedef zone: {msg.zone_color} renkli bölge\n'
            f'   Sürüden ayrılıyorum, iniş bölgesine gidiyorum...'
        )
        self._pending_detach = True
        self._transition_to(State.DETACH)
        self._send_mode('GUIDED')
        # formation_controller DETACH state'ini görünce bu drone'a
        # zone koordinatını setpoint olarak verecek (zone_color'a göre)

    def _handle_rejoin_intent(self, msg: SwarmIntent):
        """
        task_id='REJOIN': Birey ekleme görevi.
        Şartname §15: DISARM_WAIT'teki drone yeniden ARM edilip sürüye katılır.

        Sadece join_drone_id bizim ID'mize eşitse hareket ederiz.
        """
        if msg.join_drone_id != self.drone_id:
            return

        if self._state not in (State.DISARM_WAIT, State.REARM):
            return

        if self._state == State.DISARM_WAIT:
            self.get_logger().info(
                f'🔄 [{self.ns}] REARM! Sürüye yeniden katılıyorum...'
            )
            self._transition_to(State.REARM)
            # Drone'u tekrar ARM et
            self._send_arm(True)
            self._send_mode('GUIDED')

        elif self._state == State.REARM:
            # ARM tamamlandı, artık sürüye katılma manevrası
            self._transition_to(State.REJOIN)
            self.get_logger().info(
                f'🚁 [{self.ns}] REJOIN! Sürü formasyon pozisyonuna yükseliyorum...'
            )

    def _handle_return_home_intent(self, msg: SwarmIntent):
        """
        task_id='RETURN_HOME': Tüm QR görevleri tamamlandı.
        Şartname §18: Sürü formasyonu bozulmadan home'a döner ve iner.

        FLYING veya DISARM_WAIT state'indeki drone'lar bu komutu alır.
        """
        if self._state == State.FLYING:
            self.get_logger().info(
                f'🏠 [{self.ns}] HOME dönüşü başlıyor! '
                f'FLYING → RETURN_HOME'
            )
            self._transition_to(State.RETURN_HOME)
            self._send_mode('GUIDED')
            # formation_controller home koordinatını setpoint olarak verecek

        elif self._state == State.RETURN_HOME:
            # Home'a geldik → LANDING'e geç
            # Bu geçiş formation_controller'dan gelecek "home ulaşıldı" sinyaliyle
            # Şimdilik intent'te task_id değişimiyle tetikleyeceğiz
            pass

    # ══════════════════════════════════════════════════════════════════════
    # STATE GEÇİŞ ÖTESİ OLAYLAR
    # Dış trigger olmadan oluşan geçişler (örn: iniş tamamlandı)
    # ══════════════════════════════════════════════════════════════════════

    def on_detach_arrived_at_zone(self):
        """
        Dışarıdan çağrılır: formation_controller bu drone'un zone
        yakınında olduğunu tespit edince local_fsm'i bilgilendirir.
        DETACH → LAND_ZONE geçişi.

        NOT: Şu an bu geçiş formation_controller entegre edilince
        bir topic üzerinden tetiklenecek. Şimdilik placeholder.
        """
        if self._state == State.DETACH:
            self.get_logger().info(
                f'🎯 [{self.ns}] İniş bölgesine ulaşıldı! '
                f'DETACH → LAND_ZONE (precision_landing aktif)'
            )
            self._transition_to(State.LAND_ZONE)
            # precision_landing artık landing_target topic'ini yayınlayacak
            # drone_interface onu alıp Pixhawk'a iletecek

    def on_landing_complete(self):
        """
        LAND_ZONE → DISARM_WAIT geçişi: İniş tamamlandı.
        Drone yerde, DISARM bekleniyor.

        NOT: drone_interface armed=False algıladığında veya
        altitude < 0.3m olduğunda bu çağrılacak.
        Şimdilik placeholder.
        """
        if self._state == State.LAND_ZONE:
            self.get_logger().info(
                f'✅ [{self.ns}] İniş tamamlandı! LAND_ZONE → DISARM_WAIT'
            )
            self._transition_to(State.DISARM_WAIT)
            # Drone'u DISARM et
            self._send_arm(False)
            self._send_mode('LAND')
            self._waiting_for_join_signal = True

        elif self._state == State.RETURN_HOME:
            self.get_logger().info(
                f'🏁 [{self.ns}] GÖREV TAMAMLANDI! RETURN_HOME → LANDING'
            )
            self._transition_to(State.LANDING)
            self._send_mode('LAND')

        elif self._state == State.LANDING:
            self.get_logger().info(
                f'🎉 [{self.ns}] TÜM GÖREVLER BİTTİ! DISARM ediliyor...'
            )
            self._send_arm(False)

    # ══════════════════════════════════════════════════════════════════════
    # YARDIMCI FONKSİYONLAR
    # ══════════════════════════════════════════════════════════════════════

    def _transition_to(self, new_state: str):
        """
        State geçişini gerçekleştir ve logla.

        Her geçişte:
          1) State değiştirilir
          2) Log yazılır (hangi state'ten hangisine?)
          3) State anında yayınlanır (heartbeat değil, anında!)
        """
        old_state = self._state

        # Aynı state'e geçmeye çalışıyorsa gereksiz log engelle
        if old_state == new_state:
            return

        self._state = new_state
        self.get_logger().info(
            f'🔄 [{self.ns}] STATE: {old_state} → {new_state}'
        )

        # State değişti → ANINDA yayınla (heartbeat'i bekleme!)
        self._publish_state()

    def _send_mode(self, mode: str):
        """
        Pixhawk'a mod komutu gönder.
        drone_interface bunu alır ve MAVROS set_mode servisini çağırır.

        KRİTİK: Bu fonksiyon SADECE local_fsm içinde çağrılabilir!
        Başka node bu publisher'a yazamaz.
        """
        msg = String()
        msg.data = mode
        self.cmd_mode_pub.publish(msg)
        self.get_logger().info(f'📡 [{self.ns}] Mod komutu: {mode}')

    def _send_arm(self, value: bool):
        """
        Drone ARM / DISARM et.
        drone_interface bunu alır ve MAVROS arming servisini çağırır.

        ARM=True  → Motorlar haz ır, uçuşa hazır
        ARM=False → Motorlar durur, güvenli park
        """
        msg = Bool()
        msg.data = value
        self.cmd_arm_pub.publish(msg)
        action = 'ARM' if value else 'DISARM'
        self.get_logger().info(f'🔐 [{self.ns}] {action} komutu gönderildi')

    def _publish_state(self):
        """
        Mevcut state'i LocalState mesajı olarak yayınla.

        Çağrıldığı yerler:
          1) _transition_to() → state değiştiğinde ANINDA
          2) 10 Hz timer    → heartbeat (her 0.1 saniyede bir)

        NEDEN 10 Hz?
          intent_coordinator 500ms timeout kullanır.
          10 Hz → 500ms içinde 5 paket gönderilir.
          Wi-Fi jitter ile birkaç paket kaybolsa bile
          timeout dolmaz → false-positive düşme alarmı engellenir.
          (0.5 Hz ile timeout aynı değer olurdu → tek jitter = alarm!)
        """
        self._state_seq += 1
        msg = LocalState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.drone_id = self.drone_id
        msg.state = self._state
        msg.seq = self._state_seq
        self.local_state_pub.publish(msg)

    def _publish_join_request_if_standby(self):
        """
        STANDBY state'indeyse periyodik join_request yayınla.

        "Ben hazırım, ne zaman sürüye katılayım?"
        intent_coordinator bir slot açıldığında bizi çağıracak
        (SwarmIntent.join_drone_id = bizim ID).
        """
        if self._state != State.STANDBY:
            return

        msg = JoinRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drone_id = self.drone_id
        msg.battery_pct = 100.0   # TODO: gerçek batarya değeri eklenecek
        msg.ready = True
        self.join_request_pub.publish(msg)

        self.get_logger().debug(
            f'[{self.ns}] STANDBY: join_request gönderildi '
            f'(drone_id={self.drone_id}, battery=100%)'
        )

    # ══════════════════════════════════════════════════════════════════════
    # DURUM SORGULAMA YARDIMCILARI
    # ══════════════════════════════════════════════════════════════════════

    @property
    def current_state(self) -> str:
        """Mevcut state'i döndür."""
        return self._state

    def is_flying(self) -> bool:
        """Drone hav ada uçuyor mu?"""
        return self._state in (
            State.FLYING, State.DETACH, State.REJOIN,
            State.RETURN_HOME, State.LAND_ZONE
        )

    def is_autonomous(self) -> bool:
        """Drone otonom kontrolde mi? (PILOT_OVERRIDE ve SAFETY_HOLD dışında)"""
        return self._state not in (
            State.PILOT_OVERRIDE, State.SAFETY_HOLD, State.STANDBY
        )


# ══════════════════════════════════════════════════════════════════════════
# ANA FONKSİYON
# ══════════════════════════════════════════════════════════════════════════

def main(args=None):
    """
    Node başlatma.

    TEST ETME:
    ----------
    1) Simülasyon çalışıyorken:
       ros2 run my_swarm_pkg local_fsm --ros-args -p drone_id:=1

    2) State'i izle:
       ros2 topic echo /drone1/local_state

    3) Manuel intent gönder (test için):
       ros2 topic pub /swarm/intent swarm_msgs/msg/SwarmIntent \\
         "{seq: 1, leader_id: 1, task_id: 'IDLE', drone_altitude: 5.0}" --once

    4) Pilot override simüle et:
       ros2 topic pub /drone1/pilot_override std_msgs/msg/Bool "{data: true}" --once

    5) Tüm drone'ların state'ini izle:
       ros2 topic echo /drone1/local_state &
       ros2 topic echo /drone2/local_state &
       ros2 topic echo /drone3/local_state &
    """
    rclpy.init(args=args)
    node = LocalFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'⛔ local_fsm durduruluyor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
