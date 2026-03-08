#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                      waypoint_navigator.py  v1                              ║
║              Waypoint Navigatörü — QR→QR Rota Takibi                        ║
╚══════════════════════════════════════════════════════════════════════════════╝

GENEL AÇIKLAMA:
---------------
Bu node, sürünün sanal merkez noktasının (virtual centroid) bir QR'dan
sonrakine doğru ilerlemesini izler ve QR bölgesine yaklaşınca:
  1) Loiter (bekle) sinyali yayınlar → sürü mevcut konumda hover
  2) qr_perception'ı tetikler → kamerayı aktive eder
  3) Bekleme süresi dolunca → intent_coordinator'a devam sinyali gönderir

Bu node SADECE LİDER DRONE üzerinde aktif çalışır.
Diğer drone'larda subscribe eder ama işlem yapmaz (bant genişliği tasarrufu).

VERİ AKIŞI:
-----------
  /swarm/intent       ──► waypoint_navigator (hedef & task_id)
  /{ns}/pose          ──► kendi konum (lider merkez proxy)
  /drone{i}/pose      ──► tüm drone konumları (centroid hesabı)
  /drone{i}/local_state ─► FLYING drone filtreleme

  waypoint_navigator ──► /swarm/loiter_cmd  (std_msgs/Bool — sürüyü durdur)
  waypoint_navigator ──► /qr/trigger        (std_msgs/Bool — kamerayı aç)
  waypoint_navigator ──► /swarm/nav_status  (std_msgs/String — durum)

ALGORİTMA:
----------
  Her 10 Hz'de:
    1) FLYING drone'ların centroid'ini hesapla
    2) centroid → intent.target_pos mesafesini ölç
    3) d < LOITER_RADIUS_M → LOITER faz: loiter_cmd=True + qr/trigger=True
    4) LOITER_DURATION_S sonra → loiter_cmd=False (devam)

LOİTER FAZ AKIŞI:
-----------------
  d < LOITER_RADIUS_M
       ↓
  loiter_cmd = True  → formation_controller setpoint'i dondurur
  qr/trigger = True  → qr_perception kamerayı aktive eder
       ↓
  wait_seconds (QRResult.wait_seconds veya varsayılan 3s) bekle
       ↓
  loiter_cmd = False → formation_controller yeni hedefe gider
  qr/trigger = False → kamera kapanır

MESAFE HEDEFİ SINIFLARI:
--------------------------
  d > APPROACH_RADIUS_M : FAR      — tam hızda git
  d ∈ [LOITER_RADIUS, APPROACH] : APPROACH — yavaşla (formation_controller
                                               VL hızını azalt değil, biz
                                               intent'i yavaşlatmıyoruz)
  d < LOITER_RADIUS_M   : LOITER   — dur ve QR oku
  RETURN_HOME fazında   : HOME_APPROACH — eve yaklaşma

ŞARTNAME UYUMU:
  §14   QR okuma       → Trigger radius içinde QR algılama başlar ✅
  §14   Bekleme süresi → QR içindeki bekleme_suresi_s uygulanır ✅
  §5.3  Dağıtık mimari → Sadece lider waypoint kararı verir ✅
  §18   Eve dönüş      → RETURN_HOME fazında home_pos'a navige ✅
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String, Header

from swarm_msgs.msg import LocalState, SwarmIntent, QRResult

# ══════════════════════════════════════════════════════════════════════════════
# PARAMETRE SABİTLERİ
# ══════════════════════════════════════════════════════════════════════════════

CTRL_HZ: float = 10.0
CTRL_DT: float = 1.0 / CTRL_HZ

# Waypoint yaklaşma mesafeleri
LOITER_RADIUS_M: float  = 5.0   # Bu mesafede loiter başlar (QR okuma)
APPROACH_RADIUS_M: float = 20.0  # Bu mesafede "yaklaşma" moduna gir (log/debug)
HOME_LOITER_M: float    = 3.0   # Eve bu kadar yakınken hover et

# Varsayılan loiter süresi (QRResult.wait_seconds gelmezse)
DEFAULT_LOITER_S: float = 3.0

# QR trigger cooldown: bir QR'ı tekrar tetikleme süresi
QR_TRIGGER_COOLDOWN_S: float = 10.0

# Pose stale guard
POSE_STALE_S: float = 1.5

# SwarmIntent stale guard (bu kadar eski intent → navigasyonu dondur)
INTENT_STALE_S: float = 1.0

# Home koordinatı (qr_map.yaml ile uyumlu)
HOME_X: float = float(os.environ.get('HOME_X', '0.0'))
HOME_Y: float = float(os.environ.get('HOME_Y', '0.0'))
HOME_Z: float = float(os.environ.get('HOME_Z', '5.0'))

SWARM_SIZE: int = int(os.environ.get('SWARM_SIZE', '3'))


# ══════════════════════════════════════════════════════════════════════════════
# YARDIMCI FONKSİYONLAR
# ══════════════════════════════════════════════════════════════════════════════

def _dist3(a: tuple[float, float, float],
           b: tuple[float, float, float]) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


def _dist2(a: tuple[float, float, float],
           b: tuple[float, float, float]) -> float:
    """Sadece XY düzlem mesafesi (irtifa farkı göz ardı edilir)."""
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2
    )


# ══════════════════════════════════════════════════════════════════════════════
# NAVİGATÖR FAZ
# ══════════════════════════════════════════════════════════════════════════════

class NavPhase:
    IDLE         = 'IDLE'         # Görev yok
    NAVIGATING   = 'NAVIGATING'   # Hedefe gidiliyor
    APPROACHING  = 'APPROACHING'  # Hedefe yaklaşılıyor
    LOITERING    = 'LOITERING'    # QR noktasında hover
    WAITING      = 'WAITING'      # QR bekleme süresi sayılıyor
    HOME_RETURN  = 'HOME_RETURN'  # Eve dönüş navigasyonu
    COMPLETE     = 'COMPLETE'     # Görev bitti


# ══════════════════════════════════════════════════════════════════════════════
# ANA NODE
# ══════════════════════════════════════════════════════════════════════════════

class WaypointNavigatorNode(Node):
    """
    QR→QR waypoint takip node'u.

    Her 10 Hz'de sürünün centroid'ini hesaplar ve
    hedefe olan mesafeye göre loiter/devam kararı verir.
    """

    def __init__(self):
        self.ns: str       = os.environ.get('DRONE_NS', 'drone1')
        self.drone_id: int = int(os.environ.get('DRONE_ID', '1'))
        self.swarm_size    = SWARM_SIZE

        super().__init__(
            f'waypoint_navigator_{self.drone_id}',
            namespace=self.ns,
        )

        self.get_logger().info(
            f'WaypointNavigator başladı — ns={self.ns}, id={self.drone_id}'
        )

        # ── Dahili durum ──────────────────────────────────────────────────────
        self._nav_phase: str = NavPhase.IDLE

        # Aktif hedef
        self._target_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._current_task_id: str = 'IDLE'
        self._current_qr_seq: int  = 0

        # Drone konumları
        self._own_pose: tuple[float, float, float] | None = None
        self._own_pose_time: float = 0.0
        self._neighbor_poses: dict[int, tuple[float, float, float]] = {}
        self._neighbor_pose_times: dict[int, float] = {}
        self._neighbor_states: dict[int, str] = {}

        # Intent takibi
        self._last_intent_time: float = 0.0
        self._is_leader: bool = False

        # Loiter yönetimi
        self._loiter_start_time: float = 0.0
        self._loiter_duration_s: float = DEFAULT_LOITER_S
        self._loiter_active: bool      = False

        # QR trigger cooldown
        self._last_qr_trigger_time: float  = 0.0
        self._last_triggered_qr_seq: int   = -1

        # Son yayınlanan loiter durumu (gereksiz tekrar yayını önle)
        self._last_loiter_pub: bool | None = None

        # ── QoS ──────────────────────────────────────────────────────────────
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Abonelikler ───────────────────────────────────────────────────────

        # Sürü niyeti — hedef konum + task_id
        self.create_subscription(
            SwarmIntent, '/swarm/intent', self._on_intent, 10
        )

        # QRResult — bekleme süresi için
        self.create_subscription(
            QRResult, '/qr/result', self._on_qr_result, 10
        )

        # Kendi konumu
        self.create_subscription(
            PoseStamped,
            f'/{self.ns}/pose',
            self._on_own_pose,
            best_effort_qos,
        )

        # Komşu konumları ve durumları
        for i in range(1, self.swarm_size + 1):
            self.create_subscription(
                PoseStamped,
                f'/drone{i}/pose',
                lambda msg, uid=i: self._on_neighbor_pose(msg, uid),
                best_effort_qos,
            )
            self.create_subscription(
                LocalState,
                f'/drone{i}/local_state',
                lambda msg, uid=i: self._on_neighbor_state(msg, uid),
                reliable_qos,
            )

        # ── Yayıncılar ────────────────────────────────────────────────────────

        # Sürüyü dondur/serbest bırak (formation_controller dinler)
        self._loiter_pub = self.create_publisher(
            Bool, '/swarm/loiter_cmd', reliable_qos
        )

        # QR kamerasını tetikle (qr_perception dinler)
        self._qr_trigger_pub = self.create_publisher(
            Bool, '/qr/trigger', reliable_qos
        )

        # Navigatör durum özeti (mission_fsm ve debug için)
        self._status_pub = self.create_publisher(
            String, '/swarm/nav_status', 10
        )

        # ── Timer ─────────────────────────────────────────────────────────────
        self.create_timer(CTRL_DT, self._nav_loop)
        self.create_timer(1.0,     self._status_loop)

        self.get_logger().info(
            f'Parametreler: LOITER_RADIUS={LOITER_RADIUS_M}m, '
            f'DEFAULT_LOITER={DEFAULT_LOITER_S}s'
        )

    # ══════════════════════════════════════════════════════════════════════════
    # CALLBACK'LER
    # ══════════════════════════════════════════════════════════════════════════

    def _on_intent(self, msg: SwarmIntent) -> None:
        """SwarmIntent — hedef ve görev bilgisi."""
        self._last_intent_time  = time.time()
        self._is_leader         = (msg.leader_id == self.drone_id)
        self._current_task_id   = msg.task_id
        self._current_qr_seq    = int(msg.qr_seq)

        p = msg.target_pos
        self._target_xyz = (p.x, p.y, p.z)

        # Phase güncelle
        if msg.task_id in ('QR_NAVIGATE', 'MANEUVER'):
            if self._nav_phase == NavPhase.IDLE:
                self._nav_phase = NavPhase.NAVIGATING
                self.get_logger().info(
                    f'NAV BAŞLADI → hedef '
                    f'({p.x:.1f}, {p.y:.1f}, {p.z:.1f})'
                )
        elif msg.task_id == 'RETURN_HOME':
            if self._nav_phase != NavPhase.HOME_RETURN:
                self._nav_phase  = NavPhase.HOME_RETURN
                self._target_xyz = (HOME_X, HOME_Y, HOME_Z)
                self.get_logger().info(
                    f'RETURN_HOME → ev ({HOME_X}, {HOME_Y}, {HOME_Z})'
                )
        elif msg.task_id == 'IDLE':
            self._nav_phase = NavPhase.IDLE

    def _on_qr_result(self, msg: QRResult) -> None:
        """
        QR okundu — bekleme süresini güncelle.
        LOITERING fazındaki bekleme süresini QR'dan gelen değere çek.
        """
        if msg.wait_seconds > 0:
            self._loiter_duration_s = float(msg.wait_seconds)
            self.get_logger().info(
                f'QR#{msg.qr_id} bekleme süresi: {msg.wait_seconds:.1f}s'
            )

    def _on_own_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._own_pose      = (p.x, p.y, p.z)
        self._own_pose_time = time.time()
        # Kendi konumunu komşu listesine de ekle (centroid hesabı için)
        self._neighbor_poses[self.drone_id]       = self._own_pose
        self._neighbor_pose_times[self.drone_id]  = self._own_pose_time

    def _on_neighbor_pose(self, msg: PoseStamped, uid: int) -> None:
        p = msg.pose.position
        self._neighbor_poses[uid]      = (p.x, p.y, p.z)
        self._neighbor_pose_times[uid] = time.time()

    def _on_neighbor_state(self, msg: LocalState, uid: int) -> None:
        self._neighbor_states[uid] = msg.state

    # ══════════════════════════════════════════════════════════════════════════
    # CENTROİD HESABI
    # ══════════════════════════════════════════════════════════════════════════

    def _get_flying_centroid(self) -> tuple[float, float, float] | None:
        """
        FLYING/DETACH/REJOIN olan tüm drone'ların ortalama konumu.
        Yeterli veri yoksa None döner.
        """
        now = time.time()
        xs, ys, zs = [], [], []

        for uid, pose in self._neighbor_poses.items():
            # Stale kontrol
            if now - self._neighbor_pose_times.get(uid, 0.0) > POSE_STALE_S:
                continue
            # Durum filtresi
            state = self._neighbor_states.get(uid, '')
            if state not in ('FLYING', 'DETACH', 'REJOIN', 'LAND_ZONE',
                             'RETURN_HOME'):
                continue
            xs.append(pose[0])
            ys.append(pose[1])
            zs.append(pose[2])

        if not xs:
            return None

        n = len(xs)
        return (sum(xs) / n, sum(ys) / n, sum(zs) / n)

    # ══════════════════════════════════════════════════════════════════════════
    # LOITER KONTROL
    # ══════════════════════════════════════════════════════════════════════════

    def _publish_loiter(self, active: bool) -> None:
        """Loiter komutunu yayınla (değişmişse)."""
        if self._last_loiter_pub == active:
            return
        self._last_loiter_pub = active
        self._loiter_active   = active

        msg      = Bool()
        msg.data = active
        self._loiter_pub.publish(msg)

        if active:
            self.get_logger().info('🛑 LOİTER AKTİF — sürü hover')
        else:
            self.get_logger().info('▶  LOİTER BİTTİ — devam')

    def _publish_qr_trigger(self, active: bool) -> None:
        """QR kamera trigger yayınla."""
        msg      = Bool()
        msg.data = active
        self._qr_trigger_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════════════
    # ANA NAVİGASYON DÖNGÜSÜ — 10 Hz
    # ══════════════════════════════════════════════════════════════════════════

    def _nav_loop(self) -> None:
        """
        10 Hz navigasyon döngüsü.

        Sadece lider drone aktif karar verir.
        Diğer drone'lar sadece subscribe'da durur.
        """
        # ── Lider kontrolü ────────────────────────────────────────────────────
        if not self._is_leader:
            return

        # ── Intent tazeliği ───────────────────────────────────────────────────
        if time.time() - self._last_intent_time > INTENT_STALE_S:
            return  # Stale intent → bekle

        # ── Görev aktif mi? ───────────────────────────────────────────────────
        if self._nav_phase == NavPhase.IDLE:
            return

        if self._nav_phase == NavPhase.COMPLETE:
            self._publish_loiter(False)
            return

        # ── Centroid hesabı ───────────────────────────────────────────────────
        centroid = self._get_flying_centroid()
        if centroid is None:
            # Konum verisi yok → loiter güvenli
            self._publish_loiter(True)
            return

        # ── Mesafe hesabı (XY düzlemi) ────────────────────────────────────────
        d_xy = _dist2(centroid, self._target_xyz)
        # 3D mesafe (eve dönüş ve irtifa geçişleri için)
        d_3d = _dist3(centroid, self._target_xyz)

        # ══════════════════════════════════════════════════════════════════
        # RETURN_HOME FAZI
        # ══════════════════════════════════════════════════════════════════
        if self._nav_phase == NavPhase.HOME_RETURN:
            if d_xy < HOME_LOITER_M:
                # Eve geldik → loiter et
                self._publish_loiter(True)
                self._nav_phase = NavPhase.COMPLETE
                self.get_logger().info(
                    f'🏁 EVE VARILDI! d_xy={d_xy:.2f}m — '
                    f'Loiter aktif, iniş bekleniyor'
                )
            else:
                self._publish_loiter(False)
            return

        # ══════════════════════════════════════════════════════════════════
        # LOITERING / WAITING FAZI
        # ══════════════════════════════════════════════════════════════════
        if self._nav_phase in (NavPhase.LOITERING, NavPhase.WAITING):
            self._handle_loiter_phase()
            return

        # ══════════════════════════════════════════════════════════════════
        # NAVIGATING / APPROACHING FAZI
        # ══════════════════════════════════════════════════════════════════

        # Faz güncelle (debug için)
        if d_xy < LOITER_RADIUS_M:
            # Loiter bölgesine girdik
            self._enter_loiter()
        elif d_xy < APPROACH_RADIUS_M:
            if self._nav_phase != NavPhase.APPROACHING:
                self._nav_phase = NavPhase.APPROACHING
                self.get_logger().debug(
                    f'APPROACHING: d_xy={d_xy:.1f}m, '
                    f'hedef=({self._target_xyz[0]:.1f},'
                    f'{self._target_xyz[1]:.1f})'
                )
            self._publish_loiter(False)
        else:
            if self._nav_phase != NavPhase.NAVIGATING:
                self._nav_phase = NavPhase.NAVIGATING
            self._publish_loiter(False)

    def _enter_loiter(self) -> None:
        """Loiter fazına gir."""
        # Aynı QR seq için tekrar tetikleme önleme
        if (self._current_qr_seq == self._last_triggered_qr_seq
                and self._current_task_id != 'RETURN_HOME'):
            now = time.time()
            if now - self._last_qr_trigger_time < QR_TRIGGER_COOLDOWN_S:
                return  # Cooldown aktif

        self._nav_phase        = NavPhase.LOITERING
        self._loiter_start_time = time.time()
        self._loiter_duration_s = DEFAULT_LOITER_S  # QRResult gelince override edilir
        self._last_triggered_qr_seq = self._current_qr_seq
        self._last_qr_trigger_time  = time.time()

        # Loiter + QR tetikle
        self._publish_loiter(True)
        self._publish_qr_trigger(True)

        self.get_logger().info(
            f'📍 LOITER GİRİŞİ — QR#{self._current_qr_seq} '
            f'bekleme={self._loiter_duration_s:.1f}s'
        )

    def _handle_loiter_phase(self) -> None:
        """Loiter süresi kontrolü."""
        elapsed = time.time() - self._loiter_start_time

        if self._nav_phase == NavPhase.LOITERING:
            # QRResult gelmesi için kısa süre (1.5s) bekle → sonra WAITING
            if elapsed >= 1.5:
                self._nav_phase = NavPhase.WAITING
                self.get_logger().info(
                    f'QR trigger gönderildi, '
                    f'{self._loiter_duration_s:.1f}s bekleniyor...'
                )
            return

        # WAITING fazı: toplam loiter_duration_s doldu mu?
        if elapsed >= self._loiter_duration_s:
            # Loiter bitti → devam
            self._nav_phase = NavPhase.NAVIGATING
            self._publish_loiter(False)
            self._publish_qr_trigger(False)
            self.get_logger().info(
                f'✅ Bekleme tamamlandı ({elapsed:.1f}s) — '
                f'sürü bir sonraki hedefe ilerliyor'
            )

    # ══════════════════════════════════════════════════════════════════════════
    # DURUM YAYINI — 1 Hz
    # ══════════════════════════════════════════════════════════════════════════

    def _status_loop(self) -> None:
        """1 Hz — durum özeti yayınla."""
        centroid = self._get_flying_centroid()

        if centroid:
            d_xy = _dist2(centroid, self._target_xyz)
            status = (
                f'phase={self._nav_phase} '
                f'task={self._current_task_id} '
                f'qr_seq={self._current_qr_seq} '
                f'd_xy={d_xy:.1f}m '
                f'target=({self._target_xyz[0]:.1f},'
                f'{self._target_xyz[1]:.1f}) '
                f'centroid=({centroid[0]:.1f},{centroid[1]:.1f})'
            )
        else:
            status = (
                f'phase={self._nav_phase} '
                f'task={self._current_task_id} '
                f'centroid=None'
            )

        msg      = String()
        msg.data = status
        self._status_pub.publish(msg)


# ══════════════════════════════════════════════════════════════════════════════
# ENTRYPOINT
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ══════════════════════════════════════════════════════════════════════════════
# MANUEL TEST
# ══════════════════════════════════════════════════════════════════════════════
#
# Terminal 1 — navigatör (lider drone1):
#   DRONE_NS=drone1 DRONE_ID=1 SWARM_SIZE=3 \
#   ros2 run my_swarm_pkg waypoint_navigator
#
# Terminal 2 — sahte intent (QR_NAVIGATE, hedef x=30):
#   ros2 topic pub /swarm/intent swarm_msgs/msg/SwarmIntent \
#     '{leader_id: 1, task_id: "QR_NAVIGATE",
#       target_pos: {x: 30.0, y: 0.0, z: 20.0}, qr_seq: 1}' --rate 5 &
#
# Terminal 3 — sahte drone1 konumu (hedefe doğru yaklaşıyor):
#   ros2 topic pub /drone1/pose geometry_msgs/msg/PoseStamped \
#     '{pose: {position: {x: 4.0, y: 0.0, z: 20.0}}}' --rate 10 &
#   ros2 topic pub /drone1/local_state swarm_msgs/msg/LocalState \
#     '{drone_id: 1, state: "FLYING"}' --rate 5
#
# Terminal 4 — loiter ve qr trigger izle:
#   ros2 topic echo /swarm/loiter_cmd
#   ros2 topic echo /qr/trigger
#   ros2 topic echo /swarm/nav_status
#
# Beklenen: d_xy=26m → NAVIGATING, d_xy<5m → LOITERING + loiter=True + qr=True
