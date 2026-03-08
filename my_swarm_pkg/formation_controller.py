#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                       formation_controller.py  v3                           ║
║          Formasyon Kontrolcüsü — Her Drone'un Setpoint Hesaplayıcısı        ║
╚══════════════════════════════════════════════════════════════════════════════╝

GENEL AÇIKLAMA:
---------------
Bu node, sürünün geometrik konumlanmasını yönetir. Liderden gelen SwarmIntent
mesajına göre her drone'un "nereye gitmesi gerektiğini" hesaplar ve setpoint
olarak yayınlar.

v2 DEĞİŞİKLİKLERİ (Gemini eleştirileri + rehber uyumu):
  1) GERÇEK CENTROİD: /drone{i}/pose'lardan FLYING drone ortalaması hesaplanır
     → Rüzgar itiği veya hata olan durumlarda formasyon bütünlüğü korunur
  2) GÖREV-2 TELEOP: /swarm/teleop_cmd ve /swarm/teleop_mode dinlenir
     → Joystick hız komutunu kinematik entegrasyon ile setpoint'e çevirir
  3) OSİLASYON DAMPENING: /safety/event dinlenir (OSCILLATION event'i)
     → Slew rate ve hız limiti azaltılarak titreme yumuşatılır
  4) SLEW RATE LİMİTER: Setpoint ani sıçramalarını önler
     → ArduPilot'a gönderilen setpoint max_sp_change_per_step ile sınırlanır
  5) INTENT TIMEOUT: Stale (eski/kopuk) intent koruması
     → INTENT_TIMEOUT_S içinde yeni intent gelmezse setpoint dondurulur
  6) RANK GEÇİŞ KORUMASI: Detach sonrası rank yeniden hesabında
     → Önce ara waypoint kullanılır, doğrudan çapraz uçuş engellenir

v3 DEĞİŞİKLİĞİ (Gemini "Drifting Swarm / Sürüklenen Sürü" eleştirisi):
  7) SANAL LİDER — Virtual Structure: intent.target_pos'a smooth navigasyon
     → Fiziksel centroid sadece init/teleop için; navigasyon sanal lidere devredildi
     → Sanal lider her 50Hz adımında VIRTUAL_LEADER_SPEED_MPS = 3 m/s hızıyla hedefe ilerler
     → Sürü artık QR noktasına gerçekten gider; rüzgara kapılmaz!

VERİ AKIŞI:
-----------
  /drone{i}/pose  ─────────────────────────────────┐
  /swarm/intent  ──► formation_controller (BU NODE) ├──► /{ns}/setpoint_raw
  /swarm/formation_cmd                               │       ↓
  /swarm/teleop_cmd    (Görev-2)                     │   collision_avoidance
  /swarm/teleop_mode   (Görev-2)                     │       ↓
  /safety/event        (osilasyon dampening)  ────────┘   setpoint_final
                                                              ↓
                                                       drone_interface → MAVROS

NAVİGASYON (v3 — Sanal Lider / Virtual Structure):
----------------------------------------------------
  v2 HATASI : Fiziksel centroid → formasyon tutulur AMA hedefe gidilmez!
  v3 ÇÖZÜM  : Sanal lider intent.target_pos'a hareket eder; formasyon buna göre kurulur.

  virtual_centroid_xyz:
    • FLYING geçişinde fiziksel konumdan sıfırlanır
    • Her 50Hz adımında → intent.target_pos'a 3 m/s hızında ilerler
    • ARRIVAL_RADIUS_M = 0.3 m içine girince hedefte sabitlenir
    • intent.target_pos değişince (yeni QR kodu) otomatik yeni hedefe yönelir

  _get_physical_centroid(): sadece sanal lider init + teleop başlangıcı için

FORMASYON GEOMETRİSİ:
---------------------
  Yerel formasyon çerçevesi:
    +X_local = ileri (target_yaw yönü)
    +Y_local = sol
    +Z_local = yukarı

  ENU dönüşümü:
    dx = fwd * cos(yaw) - left * sin(yaw)
    dy = fwd * sin(yaw) + left * cos(yaw)

MANEVRA GEOMETRİSİ (pitch/roll):
----------------------------------
  dz = fwd  * tan(pitch_rad)   [centroid irtifası sabit: Σdz = 0]
     + left * tan(roll_rad)

SLEW RATE LİMİTER:
------------------
  Her 50 Hz adımında setpoint en fazla NORMAL_SP_CHANGE_M (= 0.1m) değişebilir.
  Normal modda: ~5 m/s max hız
  Osilasyon modunda: ~1 m/s max hız (DAMPENED_SP_CHANGE_M = 0.02m)

GÖREV-2 KİNEMATİK ENTEGRASYON:
--------------------------------
  MOVE modunda joystick vx/vy/vz/yaw_rate gelir.
  Her adımda: centroid += v * dt ve yaw += yaw_rate * dt
  Formasyon ofsetleri yeni centroid etrafında hesaplanır.

ŞARTNAME:
  §14   Formasyon tipleri      → OKBASI, V, CIZGI ✅
  §14   Formasyon değişimi     → QR veya teleop'tan dinamik değişim ✅
  §14   Manevra (pitch/roll)   → 3D rotasyon geometrisi, centroid sabit ✅
  §5.2  Görev-2 yarı otonom    → teleop_cmd/teleop_mode entegrasyonu ✅
  §5.3  Dağıtık mimari         → Her drone kendi setpointini hesaplar ✅
  §15   Fallback (2 drone)     → CIZGI 2'li forma ✅
  Ceza  Osilasyon (-10 puan)   → Oscillation dampening mekanizması ✅
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from swarm_msgs.msg import LocalState, SwarmIntent, FormationCmd, SafetyEvent, TeleopCmd

# ══════════════════════════════════════════════════════════════════════════════
# DURUM SABİTLERİ
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

    SETPOINT_STATES = frozenset({FLYING, REJOIN})
    ACTIVE_STATES   = frozenset({FLYING, REJOIN})


# ══════════════════════════════════════════════════════════════════════════════
# FORMASYON OFSETİ TABLOSU
# ══════════════════════════════════════════════════════════════════════════════
#
# (fwd_norm, left_norm): spacing=1m için normalize edilmiş ofsetler
# KRİTİK: Her n için Σ(fwd_i)/n = 0  ve  Σ(left_i)/n = 0 olmalı!

FORMATION_OFFSETS: dict[str, dict[int, list[tuple[float, float]]]] = {

    # ── OKBASI (Ok Başı / Arrow Tip) ────────────────────────────────────────
    #       ^ (ileri yön)
    #      [0]       ← uç (rank-0 = en küçük ID)
    #   [1]   [2]    ← kanatlar
    'OKBASI': {
        3: [( 2/3,   0.0), (-1/3,  -0.5), (-1/3,  +0.5)],
        2: [(+0.5,   0.0), (-0.5,   0.0)],
        1: [( 0.0,   0.0)],
    },

    # ── V (V-Formasyon) ──────────────────────────────────────────────────────
    #       ^ (ileri yön)
    #      [0]
    #   [1]     [2]   ← OKBASI'dan geniş açı
    'V': {
        3: [( 2/3,   0.0), (-1/3,  -1.0), (-1/3,  +1.0)],
        2: [(+0.5,   0.0), (-0.5,   0.0)],
        1: [( 0.0,   0.0)],
    },

    # ── CIZGI (Çizgi / Line Abreast) ────────────────────────────────────────
    #   [0] [1] [2]   ← yana sıra
    #        ^ (ileri yön)
    'CIZGI': {
        3: [( 0.0,  +1.0), ( 0.0,   0.0), ( 0.0,  -1.0)],
        2: [( 0.0,  +0.5), ( 0.0,  -0.5)],
        1: [( 0.0,   0.0)],
    },
}

_DEFAULT_FORMATION = 'OKBASI'

# ══════════════════════════════════════════════════════════════════════════════
# ZAMAN AYIMI VE LİMİT SABİTLERİ
# ══════════════════════════════════════════════════════════════════════════════

INTENT_TIMEOUT_S         = 1.5    # Bu süre intent gelmezse setpointi dondur
SETPOINT_HZ              = 50.0   # ArduPilot setpoint hızı
NORMAL_SP_CHANGE_M       = 0.10   # Normal slew rate  (50Hz×0.10 = 5 m/s max)
DAMPENED_SP_CHANGE_M     = 0.02   # Osilasyon dampening slew rate (1 m/s max)
DAMPENING_DURATION_S     = 8.0    # Osilasyon sonrası kaç saniye dampening?
RANK_HOLD_CYCLES         = 30     # Rank değişiminde ara waypoint adımı (0.6s)
VIRTUAL_LEADER_SPEED_MPS = 3.0    # Sanal lider hızı [m/s] — Virtual Structure navigasyon
ARRIVAL_RADIUS_M         = 0.3    # Bu mesafede hedefe "ulaştı" sayılır [m]


# ══════════════════════════════════════════════════════════════════════════════
# ANA NODE
# ══════════════════════════════════════════════════════════════════════════════

class FormationController(Node):
    """
    Formasyon kontrolcüsü (v2 — tam özellikli).

    Yenilikler:
      • Gerçek centroid: FLYING drone'ların /pose ortalaması
      • Görev-2 teleop: vx/vy/vz/yaw_rate kinematik entegrasyon
      • Osilasyon dampening: SafetyEvent → slew rate azaltma
      • Slew rate limiter: ani setpoint sıçramalarını engeller
      • Intent timeout: kopuk/stale intent durumunda setpoint dondur
      • Rank geçiş koruması: ara waypoint ile yumuşak slot değişimi
    """

    def __init__(self):
        super().__init__('formation_controller')

        # ══════════════════════════════════════════════════════
        # PARAMETRELER
        # ══════════════════════════════════════════════════════
        self.declare_parameter('drone_id',   1)
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('bypass_ca',  False)

        self.drone_id   = self.get_parameter('drone_id').value
        self.num_drones = self.get_parameter('num_drones').value
        self.bypass_ca  = self.get_parameter('bypass_ca').value
        self.ns         = f'drone{self.drone_id}'

        self.get_logger().info(
            f'📐 formation_controller v2 başlatılıyor...\n'
            f'   Drone ID     : {self.drone_id}\n'
            f'   Toplam drone : {self.num_drones}\n'
            f'   CA bypass    : {"EVET (test modu)" if self.bypass_ca else "HAYIR (normal)"}'
        )

        # ══════════════════════════════════════════════════════
        # DURUM DEĞİŞKENLERİ
        # ══════════════════════════════════════════════════════

        self._latest_intent:    SwarmIntent   | None = None
        self._intent_recv_time: float               = 0.0
        self._latest_fcmd:      FormationCmd  | None = None

        self._drone_states: dict[int, str]              = {
            i: DroneState.STANDBY for i in range(1, self.num_drones + 1)
        }

        # [YENİ] Gerçek pose'lar — centroid hesabı için
        self._drone_poses:     dict[int, PoseStamped | None] = {
            i: None for i in range(1, self.num_drones + 1)
        }
        self._pose_recv_time:  dict[int, float] = {
            i: 0.0 for i in range(1, self.num_drones + 1)
        }
        self._POSE_STALE_S = 1.0   # Bu kadar süre içinde gelmezse stale

        # [YENİ] Slew rate için önceki setpoint
        self._prev_setpoint_xyz: tuple[float, float, float] | None = None

        # [YENİ] Osilasyon dampening
        self._dampening_until: float = 0.0
        self._is_dampened:     bool  = False

        # [YENİ] Rank geçiş koruması
        self._prev_rank:               int | None = None
        self._rank_change_countdown:   int        = 0

        # [v3] Sanal Lider (Virtual Structure) — navigasyon
        self._virtual_centroid_xyz: tuple[float, float, float] | None = None

        # [waypoint_navigator] Loiter komutu: True iken setpoint dondurulur
        self._loiter_active: bool = False

        # [YENİ] Görev-2 Teleop
        self._teleop_mode:     str                       = 'IDLE'
        self._teleop_centroid: tuple[float,float,float] | None = None
        self._teleop_yaw:      float                     = 0.0
        self._teleop_formation: str   | None             = None
        self._teleop_spacing:   float | None             = None
        self._latest_teleop:    TeleopCmd | None         = None
        self._last_teleop_time: float                    = 0.0

        # ══════════════════════════════════════════════════════
        # PUBLISHER'LAR
        # ══════════════════════════════════════════════════════

        self.setpoint_raw_pub = self.create_publisher(
            PoseStamped, f'/{self.ns}/setpoint_raw', 10
        )

        # [v3] Sanal lider broadcast: lider yayınlar, diğerleri dinler
        self._vl_pub = self.create_publisher(
            PoseStamped, '/swarm/virtual_leader', 10
        )
        self._shared_vl_xyz: tuple[float, float, float] | None = None
        self._shared_vl_recv_time: float = 0.0
        self._VL_STALE_S: float = 0.2  # Bu kadar eski VL → kendi hesapla

        self.setpoint_final_pub = None
        if self.bypass_ca:
            self.setpoint_final_pub = self.create_publisher(
                PoseStamped, f'/{self.ns}/setpoint_final', 10
            )
            self.get_logger().warn(f'⚠️  [{self.ns}] CA BYPASS aktif!')

        # ══════════════════════════════════════════════════════
        # ABONELİKLER
        # ══════════════════════════════════════════════════════

        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 1) Sürü niyeti
        self.create_subscription(SwarmIntent, '/swarm/intent', self._on_intent, 10)

        # 2) Formasyon komutu
        self.create_subscription(FormationCmd, '/swarm/formation_cmd', self._on_formation_cmd, 10)

        # 3) Tüm drone state'leri
        for i in range(1, self.num_drones + 1):
            self.create_subscription(
                LocalState, f'/drone{i}/local_state',
                lambda msg, did=i: self._on_local_state(msg, did), 10
            )

        # 4) [YENİ] Tüm drone'ların GERÇEK POZİSYONU — centroid için!
        for i in range(1, self.num_drones + 1):
            self.create_subscription(
                PoseStamped, f'/drone{i}/pose',
                lambda msg, did=i: self._on_drone_pose(msg, did),
                best_effort_qos
            )

        # 5) [YENİ] Görev-2 Teleop komutu
        self.create_subscription(TeleopCmd, '/swarm/teleop_cmd', self._on_teleop_cmd, 10)

        # 6) [YENİ] Teleop modu (IDLE / MOVE / MANEUVER)
        self.create_subscription(String, '/swarm/teleop_mode', self._on_teleop_mode, 10)

        # 7) [YENİ] Güvenlik olayları (osilasyon dampening)
        self.create_subscription(SafetyEvent, '/safety/event', self._on_safety_event, 10)

        # 8) [v3] Lider FC'den sanal lider pozisyonu — senkronizasyon için
        self.create_subscription(
            PoseStamped, '/swarm/virtual_leader',
            self._on_virtual_leader, best_effort_qos
        )

        # 9) [waypoint_navigator] Loiter komutu — True iken setpoint dondurulur
        from std_msgs.msg import Bool as _Bool
        self.create_subscription(
            _Bool, '/swarm/loiter_cmd', self._on_loiter_cmd, 10
        )

        # ══════════════════════════════════════════════════════
        # PERİYODİK GÖREV
        # ══════════════════════════════════════════════════════

        self.create_timer(1.0 / SETPOINT_HZ, self._publish_setpoint)

        self.get_logger().info(f'✅ [{self.ns}] formation_controller v2 HAZIR!')

    # ══════════════════════════════════════════════════════════════════════
    # CALLBACK'LER
    # ══════════════════════════════════════════════════════════════════════

    def _on_intent(self, msg: SwarmIntent):
        self._latest_intent    = msg
        self._intent_recv_time = time.time()

    def _on_virtual_leader(self, msg: PoseStamped):
        """Lider FC'den gelen sanal lider pozisyonunu kaydet."""
        p = msg.pose.position
        self._shared_vl_xyz = (p.x, p.y, p.z)
        self._shared_vl_recv_time = time.time()

    def _on_loiter_cmd(self, msg) -> None:
        """waypoint_navigator'dan gelen loiter komutu."""
        prev = self._loiter_active
        self._loiter_active = bool(msg.data)
        if prev != self._loiter_active:
            self.get_logger().info(
                f'[{self.ns}] Loiter: {"AKTİF — setpoint donduruldu" if self._loiter_active else "PASIF — devam"}'
            )

    def _on_formation_cmd(self, msg: FormationCmd):
        self._latest_fcmd = msg

    def _on_local_state(self, msg: LocalState, source_id: int):
        old_state = self._drone_states.get(source_id, DroneState.STANDBY)
        self._drone_states[source_id] = msg.state
        # Sanal lider sıfırla: kendi drone'umuz aktif uçuşa ilk geçişte
        if (source_id == self.drone_id
                and old_state not in DroneState.ACTIVE_STATES
                and msg.state in DroneState.ACTIVE_STATES):
            self._virtual_centroid_xyz = None
            self.get_logger().info(f'[{self.ns}] 🔄 Sanal lider sıfırlandı (FLYING geçişi)')

    def _on_drone_pose(self, msg: PoseStamped, source_id: int):
        """[YENİ] Gerçek pozisyon güncelleme — centroid hesabı için."""
        self._drone_poses[source_id]    = msg
        self._pose_recv_time[source_id] = time.time()

    def _on_teleop_cmd(self, msg: TeleopCmd):
        """
        [YENİ] Görev-2 joystick komutu.

        MOVE    : vx/vy/vz/yaw_rate → centroid kinematik entegrasyon
        MANEUVER: new_formation_type/new_spacing → forma değişimi, centroid sabit
        """
        if self._teleop_mode == 'IDLE':
            return

        self._latest_teleop    = msg
        self._last_teleop_time = time.time()

        if msg.mode == 'MANEUVER':
            if msg.new_formation_type:
                self._teleop_formation = msg.new_formation_type
            if msg.new_spacing > 0.0:
                self._teleop_spacing = msg.new_spacing

    def _on_teleop_mode(self, msg: String):
        """[YENİ] Teleop mod değişimi (TASK1_ACTIVE'de bu mesaj gelmez)."""
        new_mode = msg.data.upper()
        if self._teleop_mode != new_mode:
            self.get_logger().info(f'[{self.ns}] Teleop mod: {self._teleop_mode} → {new_mode}')
            self._teleop_mode = new_mode

            if new_mode == 'IDLE':
                self._teleop_centroid  = None
                self._teleop_formation = None
                self._teleop_spacing   = None
            elif new_mode in ('MOVE', 'MANEUVER'):
                if self._teleop_centroid is None:
                    cx, cy, cz = self._get_physical_centroid()
                    self._teleop_centroid = (cx, cy, cz)
                    if self._latest_intent:
                        self._teleop_yaw = self._latest_intent.target_yaw

    def _on_safety_event(self, msg: SafetyEvent):
        """
        [YENİ] Güvenlik olayı — osilasyon dampening.

        Sadece kendi drone'umuzdaki OSCILLATION event'i tetikler.
        Slew rate DAMPENED_SP_CHANGE_M'e iner, DAMPENING_DURATION_S sonra normale döner.
        """
        if msg.event_type == 'OSCILLATION' and msg.drone_id == self.drone_id:
            self._dampening_until = time.time() + DAMPENING_DURATION_S
            self._is_dampened     = True
            self.get_logger().warn(
                f'[{self.ns}] ⚠️  OSİLASYON! {DAMPENING_DURATION_S}s dampening aktif.'
            )

    # ══════════════════════════════════════════════════════════════════════
    # SETPOINT HESAPLAMA VE YAYINLAMA
    # ══════════════════════════════════════════════════════════════════════

    def _publish_setpoint(self):
        """50 Hz timer callback."""
        # Dampening güncelle
        if self._is_dampened and time.time() > self._dampening_until:
            self._is_dampened = False
            self.get_logger().info(f'[{self.ns}] ✅ Dampening bitti.')

        # Loiter aktifse: setpoint yayınlama (hover: MAVROS son setpointi tutar)
        if self._loiter_active:
            return

        # Teleop kinematik güncelleme
        if self._teleop_mode == 'MOVE' and self._latest_teleop is not None:
            self._update_teleop_centroid()

        sp = self._compute_setpoint()
        if sp is None:
            return

        sp = self._apply_slew_rate(sp)

        self.setpoint_raw_pub.publish(sp)
        if self.bypass_ca and self.setpoint_final_pub:
            self.setpoint_final_pub.publish(sp)

    def _compute_setpoint(self) -> PoseStamped | None:
        """
        Bu drone için hedef pozisyon hesapla.

        ADIMLAR:
          1) State kontrolü (FLYING/REJOIN mi?)
          2) Detach kontrolü
          3) Intent timeout (stale → dondur)
          4) Centroid al (GERÇEK pose ortalaması)
          5) Yaw/formation/spacing belirle
          6) Rank hesapla + geçiş koruması
          7) Offset hesapla + ENU dönüşümü
          8) Manevra Z ofseti
          9) PoseStamped oluştur
        """
        # 1) State
        my_state = self._drone_states.get(self.drone_id, DroneState.STANDBY)
        if my_state not in DroneState.SETPOINT_STATES:
            return None

        intent = self._latest_intent
        if intent is None:
            return None

        # 2) Detach
        if intent.task_id == 'DETACH' and intent.detach_drone_id == self.drone_id:
            return None   # precision_landing yönetiyor

        # 3) Intent timeout
        intent_age = time.time() - self._intent_recv_time
        if intent_age > INTENT_TIMEOUT_S:
            if self._prev_setpoint_xyz is not None:
                self.get_logger().warn(
                    f'[{self.ns}] Intent timeout ({intent_age:.1f}s) → donduruldu.',
                    throttle_duration_sec=3.0,
                )
                cx, cy, cz = self._prev_setpoint_xyz
                sp = PoseStamped()
                sp.header.stamp    = self.get_clock().now().to_msg()
                sp.header.frame_id = 'map'
                sp.pose.position.x = cx
                sp.pose.position.y = cy
                sp.pose.position.z = cz
                sp.pose.orientation = _yaw_to_quaternion(intent.target_yaw)
                return sp
            return None

        # 4) Sanal Lider (Virtual Structure) — hedefe doğru smooth navigasyon [v3 FIX]
        cx, cy, cz = self._get_virtual_centroid(intent)

        # 5) Yaw / formation / spacing
        if self._teleop_mode in ('MOVE', 'MANEUVER') and self._teleop_centroid:
            cx, cy, cz = self._teleop_centroid
            formation  = self._teleop_formation or intent.formation_type
            spacing    = self._teleop_spacing   or intent.drone_spacing
            yaw        = self._teleop_yaw
        elif self._latest_fcmd is not None and self._latest_fcmd.formation_type:
            formation = self._latest_fcmd.formation_type
            spacing   = self._latest_fcmd.drone_spacing
            yaw       = self._latest_fcmd.target_yaw
            cz        = intent.drone_altitude
        else:
            formation = intent.formation_type
            spacing   = intent.drone_spacing
            yaw       = intent.target_yaw
            cz        = intent.drone_altitude

        if formation not in FORMATION_OFFSETS:
            self.get_logger().warn(
                f'[{self.ns}] Bilinmeyen formasyon "{formation}" → "{_DEFAULT_FORMATION}"',
                throttle_duration_sec=5.0,
            )
            formation = _DEFAULT_FORMATION

        # 6) Rank + geçiş koruması
        active_ids = self._get_active_ids(exclude_detach=intent.detach_drone_id)
        if self.drone_id not in active_ids:
            return None

        new_rank = active_ids.index(self.drone_id)
        n        = len(active_ids)

        if self._prev_rank is not None and self._prev_rank != new_rank:
            if self._rank_change_countdown <= 0:
                self._rank_change_countdown = RANK_HOLD_CYCLES

        if self._rank_change_countdown > 0:
            self._rank_change_countdown -= 1
            rank_to_use = self._prev_rank   # Eski rank'ı koru → yumuşak geçiş
        else:
            rank_to_use = new_rank

        self._prev_rank = new_rank

        # 7) Offset + ENU dönüşümü
        available_ns = sorted(FORMATION_OFFSETS[formation].keys())
        clamp_n      = min(n, max(available_ns))
        clamp_n      = max(clamp_n, 1)
        safe_rank    = min(rank_to_use, clamp_n - 1)

        fwd_norm, left_norm = FORMATION_OFFSETS[formation][clamp_n][safe_rank]
        s    = max(spacing, 1.0)
        fwd  = fwd_norm  * s
        left = left_norm * s

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        dx = fwd * cos_y - left * sin_y
        dy = fwd * sin_y + left * cos_y

        # 8) Manevra Z
        dz = 0.0
        if intent.maneuver_active:
            pitch_rad = math.radians(intent.maneuver_pitch_deg)
            roll_rad  = math.radians(intent.maneuver_roll_deg)
            TAN_LIMIT = 1.0
            dz = (fwd  * max(-TAN_LIMIT, min(TAN_LIMIT, math.tan(pitch_rad))) +
                  left * max(-TAN_LIMIT, min(TAN_LIMIT, math.tan(roll_rad))))

        # 9) PoseStamped
        sp = PoseStamped()
        sp.header.stamp    = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = cx + dx
        sp.pose.position.y = cy + dy
        sp.pose.position.z = cz + dz
        sp.pose.orientation = _yaw_to_quaternion(yaw)
        return sp

    def _apply_slew_rate(self, sp: PoseStamped) -> PoseStamped:
        """
        [YENİ] Slew rate limiter.

        Setpoint değişimini NORMAL_SP_CHANGE_M (veya DAMPENED) ile sınırla.
        → ArduPilot'a ani sıçrama gönderilmez → osilasyon riski azalır.
        """
        nx = sp.pose.position.x
        ny = sp.pose.position.y
        nz = sp.pose.position.z

        if self._prev_setpoint_xyz is None:
            self._prev_setpoint_xyz = (nx, ny, nz)
            return sp

        px, py, pz = self._prev_setpoint_xyz
        dx = nx - px
        dy = ny - py
        dz = nz - pz
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        max_change = DAMPENED_SP_CHANGE_M if self._is_dampened else NORMAL_SP_CHANGE_M

        if dist > max_change and dist > 1e-6:
            factor = max_change / dist
            nx = px + dx * factor
            ny = py + dy * factor
            nz = pz + dz * factor
            sp.pose.position.x = nx
            sp.pose.position.y = ny
            sp.pose.position.z = nz

        self._prev_setpoint_xyz = (nx, ny, nz)
        return sp

    # ══════════════════════════════════════════════════════════════════════
    # CENTROİD HESABI: FİZİKSEL (init/teleop) + SANAL LİDER (navigasyon)
    # ══════════════════════════════════════════════════════════════════════

    def _get_physical_centroid(self) -> tuple[float, float, float]:
        """
        FLYING drone'ların GERÇEK POZİSYON ortalamasını hesapla.

        Birincil : /drone{i}/pose verilerinin ENU ortalaması
        Fallback : intent.target_pos (pose verisi yok veya stale ise)

        KULLANIM: Sadece sanal lider ilk başlatmasında ve teleop init'te.
        NAVİGASYON için → _get_virtual_centroid() kullanılır (v3).
        """
        now = time.time()
        valid: list[tuple[float, float, float]] = []

        for did in range(1, self.num_drones + 1):
            if self._drone_states.get(did) not in DroneState.ACTIVE_STATES:
                continue
            pose = self._drone_poses.get(did)
            if pose is None:
                continue
            if now - self._pose_recv_time.get(did, 0.0) > self._POSE_STALE_S:
                continue
            valid.append((
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ))

        if not valid:
            # Fallback: intent.target_pos
            if self._latest_intent is not None:
                tp = self._latest_intent.target_pos
                return (tp.x, tp.y, self._latest_intent.drone_altitude)
            return (0.0, 0.0, 5.0)

        cx = sum(p[0] for p in valid) / len(valid)
        cy = sum(p[1] for p in valid) / len(valid)
        cz = sum(p[2] for p in valid) / len(valid)
        return (cx, cy, cz)

    def _get_virtual_centroid(self, intent: SwarmIntent) -> tuple[float, float, float]:
        """
        [v3 — Virtual Structure / Sanal Lider] Navigasyon centroid'i.

        Gemini'nin "Drifting Swarm" (Sürüklenen Sürü) tuzağının çözümü:
          PROBLEM  : _get_physical_centroid() → formasyon tutulur AMA hedefe gidilmez
          ÇÖZÜM    : Sanal lider intent.target_pos'a hareket eder;
                     formasyon ofsetleri bu sanal lidere göre hesaplanır.

        Algoritma (Virtual Structure — formasyon kontrol literatürü standardı):
          1) FLYING geçişinde _on_local_state tarafından None'a sıfırlanır
          2) İlk çağrıda → _get_physical_centroid() ile fiziksel konumdan başlatılır
          3) Her 50Hz adımında → hedefe max VIRTUAL_LEADER_SPEED_MPS hızıyla ilerler
          4) ARRIVAL_RADIUS_M içine girince → hedefte sabitlenir
          5) intent.target_pos değişince (yeni QR kodu) → otomatik yeni hedefe yönelir
        """
        target_x = intent.target_pos.x
        target_y = intent.target_pos.y
        target_z = intent.drone_altitude

        # İlk çağrı: fiziksel konumdan başlat
        if self._virtual_centroid_xyz is None:
            phys = self._get_physical_centroid()
            self._virtual_centroid_xyz = phys
            self.get_logger().info(
                f'[{self.ns}] 🚀 Sanal lider başlatıldı: '
                f'({phys[0]:.1f}, {phys[1]:.1f}, {phys[2]:.1f}) → '
                f'hedef ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})'
            )

        # ── Lider değilse: paylaşılan VL'i kullan (senkronizasyon) ──────
        is_leader = (
            self._latest_intent is not None
            and self.drone_id == self._latest_intent.leader_id
        )
        shared_ok = (
            self._shared_vl_xyz is not None
            and not is_leader
            and (time.time() - self._shared_vl_recv_time) < self._VL_STALE_S
        )
        if shared_ok:
            return self._shared_vl_xyz  # type: ignore[return-value]

        # ── Kendi VL'ini hesapla (lider veya shared stale) ───────────────
        vx, vy, vz = self._virtual_centroid_xyz
        dx = target_x - vx
        dy = target_y - vy
        dz = target_z - vz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < ARRIVAL_RADIUS_M:
            self._virtual_centroid_xyz = (target_x, target_y, target_z)
        else:
            max_step = VIRTUAL_LEADER_SPEED_MPS / SETPOINT_HZ
            step     = min(max_step, dist)
            factor   = step / dist
            self._virtual_centroid_xyz = (
                vx + dx * factor,
                vy + dy * factor,
                vz + dz * factor,
            )

        # ── Lider ise: paylaşılan VL topic'ine yayınla ───────────────────
        if is_leader:
            vl_msg = PoseStamped()
            vl_msg.header.stamp    = self.get_clock().now().to_msg()
            vl_msg.header.frame_id = 'map'
            vx2, vy2, vz2 = self._virtual_centroid_xyz
            vl_msg.pose.position.x = vx2
            vl_msg.pose.position.y = vy2
            vl_msg.pose.position.z = vz2
            self._vl_pub.publish(vl_msg)

        return self._virtual_centroid_xyz

    # ══════════════════════════════════════════════════════════════════════
    # GÖREV-2 KİNEMATİK ENTEGRASYON (YENİ)
    # ══════════════════════════════════════════════════════════════════════

    def _update_teleop_centroid(self):
        """
        [YENİ] MOVE modunda joystick vx/vy/vz → centroid konum güncellemesi.

        Formation-local → ENU dönüşümü:
          dx_ENU = (vx*cos(yaw) - vy*sin(yaw)) * dt
          dy_ENU = (vx*sin(yaw) + vy*cos(yaw)) * dt
          dz_ENU = vz * dt
        """
        if self._latest_teleop is None or self._teleop_centroid is None:
            return
        if time.time() - self._last_teleop_time > 1.0:
            return   # Stale teleop → yoksay

        cmd = self._latest_teleop
        dt  = 1.0 / SETPOINT_HZ
        yaw = self._teleop_yaw

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        dx = (cmd.vx * cos_y - cmd.vy * sin_y) * dt
        dy = (cmd.vx * sin_y + cmd.vy * cos_y) * dt
        dz = cmd.vz * dt

        cx, cy, cz = self._teleop_centroid
        self._teleop_centroid = (cx + dx, cy + dy, cz + dz)
        self._teleop_yaw     += cmd.yaw_rate * dt
        self._teleop_yaw      = ((self._teleop_yaw + math.pi) % (2*math.pi)) - math.pi

    # ══════════════════════════════════════════════════════════════════════
    # YARDIMCI
    # ══════════════════════════════════════════════════════════════════════

    def _get_active_ids(self, exclude_detach: int = 0) -> list[int]:
        """FLYING+REJOIN drone'lar, exclude_detach hariç, sıralı."""
        return sorted(
            did for did in range(1, self.num_drones + 1)
            if self._drone_states.get(did) in DroneState.ACTIVE_STATES
            and did != exclude_detach
        )


# ══════════════════════════════════════════════════════════════════════════════
# GLOBAL YARDIMCILAR (unit test erişimi için)
# ══════════════════════════════════════════════════════════════════════════════

def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Yaw → Quaternion (Z ekseni, ENU çerçevesinde)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def compute_formation_offset(
    rank: int, n_drones: int, formation_type: str, spacing: float, yaw: float,
) -> tuple[float, float, float]:
    """Formasyon ofseti hesapla — unit test için global fonksiyon."""
    if formation_type not in FORMATION_OFFSETS:
        formation_type = _DEFAULT_FORMATION
    available_ns = sorted(FORMATION_OFFSETS[formation_type].keys())
    clamp_n   = min(n_drones, max(available_ns))
    clamp_n   = max(clamp_n, 1)
    safe_rank = min(rank, clamp_n - 1)
    fwd_norm, left_norm = FORMATION_OFFSETS[formation_type][clamp_n][safe_rank]
    s    = max(spacing, 1.0)
    fwd  = fwd_norm * s;  left = left_norm * s
    cos_y = math.cos(yaw);  sin_y = math.sin(yaw)
    return (fwd*cos_y - left*sin_y, fwd*sin_y + left*cos_y, 0.0)


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    """
    NORMAL: ros2 run my_swarm_pkg formation_controller --ros-args -p drone_id:=1
    TEST  : ros2 run my_swarm_pkg formation_controller --ros-args -p drone_id:=1 -p bypass_ca:=true

    GÖREV-2 TEST:
      ros2 topic pub /swarm/teleop_mode std_msgs/msg/String "{data: 'MOVE'}" --once
      ros2 topic pub /swarm/teleop_cmd swarm_msgs/msg/TeleopCmd \
          "{mode: 'MOVE', vx: 1.0, vy: 0.0, vz: 0.0, yaw_rate: 0.0}" -r 10

    OSİLASYON TEST:
      ros2 topic pub /safety/event swarm_msgs/msg/SafetyEvent \
          "{drone_id: 1, event_type: 'OSCILLATION', severity: 0.8}" --once
    """
    rclpy.init(args=args)
    node = FormationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⛔ formation_controller durduruluyor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
