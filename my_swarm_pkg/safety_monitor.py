#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                       safety_monitor.py  v1                                 ║
║              Güvenlik İzleme — Mock Batarya / Geofence / Heartbeat          ║
╚══════════════════════════════════════════════════════════════════════════════╝

GENEL AÇIKLAMA:
---------------
Bu node her drone'da çalışır ve üç kritik güvenlik durumunu izler:

  1) MOCK BATARYA   — Simülasyonda gerçek batarya yok.
     100%'den başlar, sabit hızda (0.01%/s) azalır.
     %15 eşiğine düşünce BATTERY_CRITICAL eventi yayınlanır.
     local_fsm bu eventi alınca SAFETY_HOLD'a geçer → RTL.

  2) GEOFENCE       — Drone izin verilen uçuş alanı dışına çıkarsa.
     ENU koordinat sınırları: x∈[FENCE_X_MIN, FENCE_X_MAX],
                               y∈[FENCE_Y_MIN, FENCE_Y_MAX],
                               z∈[0, FENCE_ALT_MAX]
     İhlalde GEOFENCE_BREACH eventi yayınlanır.

  3) HEARTBEAT      — intent_coordinator'dan SwarmIntent gelip gelmediğini izle.
     HEARTBEAT_TIMEOUT_S içinde mesaj gelmezse GNSS_DEGRADED değil,
     bu node'da log yeter (local_fsm intent timeout'u kendi yapar).

YAYINLANAN:
-----------
  /safety/event        → SafetyEvent (local_fsm + formation_controller)
  /{ns}/battery_pct    → Float32 (mission_fsm dashboard için)

İZLENEN:
---------
  /{ns}/pose           → Kendi konumu (geofence kontrolü)
  /swarm/intent        → Heartbeat referansı

ŞARTNAME UYUMU:
  §5.5.1 Batarya failsafe     → %15 altında BATTERY_CRITICAL ✅
  §5.5.2 Geofence             → ENU sınır ihlali → GEOFENCE_BREACH ✅
  §5.5.4 Bağımsız failsafe    → GCS'ye sormadan local karar ✅
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Header
from swarm_msgs.msg import SafetyEvent, SwarmIntent

# ══════════════════════════════════════════════════════════════════════════════
# PARAMETRE SABİTLERİ
# ══════════════════════════════════════════════════════════════════════════════

MONITOR_HZ: float = 5.0
MONITOR_DT: float = 1.0 / MONITOR_HZ

# Mock batarya
BATTERY_INITIAL_PCT: float  = 100.0
BATTERY_DRAIN_PER_S: float  = 0.01     # %/s — 100% → %15 ≈ 141 dakika
BATTERY_WARN_PCT: float     = 25.0     # Uyarı eşiği (log)
BATTERY_CRITICAL_PCT: float = 15.0     # Failsafe eşiği → SafetyEvent
BATTERY_EVENT_COOLDOWN_S: float = 30.0 # Aynı event tekrar tetikleme süresi

# Geofence sınırları — world_base.sdf ile uyumlu (ENU, metre)
FENCE_X_MIN: float = float(os.environ.get('FENCE_X_MIN', '-5.0'))
FENCE_X_MAX: float = float(os.environ.get('FENCE_X_MAX', '125.0'))
FENCE_Y_MIN: float = float(os.environ.get('FENCE_Y_MIN', '-5.0'))
FENCE_Y_MAX: float = float(os.environ.get('FENCE_Y_MAX', '95.0'))
FENCE_ALT_MAX: float = float(os.environ.get('FENCE_ALT_MAX', '60.0'))
FENCE_ALT_MIN: float = float(os.environ.get('FENCE_ALT_MIN', '0.5'))
FENCE_MARGIN_M: float = 3.0    # Sınırdan bu kadar önce uyarı ver (log)
FENCE_EVENT_COOLDOWN_S: float = 5.0

# Heartbeat
HEARTBEAT_TIMEOUT_S: float = 3.0  # Bu süre intent gelmezse uyarı logla

# Yükseklik tavan uyarısı (event'ten farklı — sadece log)
ALT_WARN_M: float = float(os.environ.get('FENCE_ALT_MAX', '60.0')) - 5.0


# ══════════════════════════════════════════════════════════════════════════════
# ANA NODE
# ══════════════════════════════════════════════════════════════════════════════

class SafetyMonitorNode(Node):
    """
    Güvenlik izleme node'u.
    Her drone'da bağımsız çalışır — GCS'ye bağımlılık yok.
    """

    def __init__(self):
        self.ns: str       = os.environ.get('DRONE_NS', 'drone1')
        self.drone_id: int = int(os.environ.get('DRONE_ID', '1'))

        super().__init__(
            f'safety_monitor_{self.drone_id}',
            namespace=self.ns,
        )

        self.get_logger().info(
            f'SafetyMonitor başladı — ns={self.ns}, id={self.drone_id}\n'
            f'  Geofence: x∈[{FENCE_X_MIN},{FENCE_X_MAX}] '
            f'y∈[{FENCE_Y_MIN},{FENCE_Y_MAX}] alt∈[{FENCE_ALT_MIN},{FENCE_ALT_MAX}]m\n'
            f'  Batarya drain: {BATTERY_DRAIN_PER_S}%/s, '
            f'kritik eşiği: %{BATTERY_CRITICAL_PCT}'
        )

        # ── Dahili durum ──────────────────────────────────────────────────────
        self._battery_pct: float  = BATTERY_INITIAL_PCT
        self._battery_start: float = time.time()

        self._own_pose: tuple[float, float, float] | None = None
        self._own_pose_time: float = 0.0

        self._last_intent_time: float = 0.0

        # Event cooldown takibi
        self._last_battery_event_time: float  = 0.0
        self._last_geofence_event_time: float = 0.0
        self._battery_event_sent: bool        = False  # Kritik event sadece 1 kez

        # Uyarı logları cooldown
        self._last_battery_warn_log: float  = 0.0
        self._last_geofence_warn_log: float = 0.0
        self._last_heartbeat_warn_log: float = 0.0

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
        self.create_subscription(
            PoseStamped,
            f'/{self.ns}/pose',
            self._on_pose,
            best_effort_qos,
        )
        self.create_subscription(
            SwarmIntent,
            '/swarm/intent',
            self._on_intent,
            10,
        )

        # ── Yayıncılar ────────────────────────────────────────────────────────
        self._safety_pub = self.create_publisher(
            SafetyEvent, '/safety/event', reliable_qos
        )
        self._battery_pub = self.create_publisher(
            Float32, f'/{self.ns}/battery_pct', 10
        )

        # ── Timer'lar ─────────────────────────────────────────────────────────
        self.create_timer(MONITOR_DT,  self._monitor_loop)
        self.create_timer(10.0,        self._battery_log_tick)

    # ══════════════════════════════════════════════════════════════════════════
    # CALLBACK'LER
    # ══════════════════════════════════════════════════════════════════════════

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._own_pose      = (p.x, p.y, p.z)
        self._own_pose_time = time.time()

    def _on_intent(self, msg: SwarmIntent) -> None:
        self._last_intent_time = time.time()

    # ══════════════════════════════════════════════════════════════════════════
    # ANA İZLEME DÖNGÜSÜ — 5 Hz
    # ══════════════════════════════════════════════════════════════════════════

    def _monitor_loop(self) -> None:
        """5 Hz — batarya + geofence + heartbeat kontrol."""
        now = time.time()

        # ── 1) Mock batarya güncelle ──────────────────────────────────────────
        elapsed = now - self._battery_start
        self._battery_pct = max(
            0.0,
            BATTERY_INITIAL_PCT - elapsed * BATTERY_DRAIN_PER_S
        )

        # Batarya yayınla
        bat_msg      = Float32()
        bat_msg.data = self._battery_pct
        self._battery_pub.publish(bat_msg)

        # ── 2) Batarya kritik kontrolü ────────────────────────────────────────
        self._check_battery(now)

        # ── 3) Geofence kontrolü ─────────────────────────────────────────────
        self._check_geofence(now)

        # ── 4) Heartbeat kontrolü ─────────────────────────────────────────────
        self._check_heartbeat(now)

    # ══════════════════════════════════════════════════════════════════════════
    # BATARYA KONTROLÜ
    # ══════════════════════════════════════════════════════════════════════════

    def _check_battery(self, now: float) -> None:
        pct = self._battery_pct

        # Uyarı logu (25% altında, 30s'de bir)
        if (pct < BATTERY_WARN_PCT
                and now - self._last_battery_warn_log > 30.0):
            self._last_battery_warn_log = now
            self.get_logger().warn(
                f'⚡ [{self.ns}] Batarya düşük: %{pct:.1f} '
                f'(uyarı eşiği: %{BATTERY_WARN_PCT})'
            )

        # Kritik event (sadece 1 kez gönder + cooldown)
        if pct > BATTERY_CRITICAL_PCT:
            self._battery_event_sent = False  # Sıfırla (şarj simülasyonu için)
            return

        if self._battery_event_sent:
            return

        if now - self._last_battery_event_time < BATTERY_EVENT_COOLDOWN_S:
            return

        self._battery_event_sent      = True
        self._last_battery_event_time = now

        evt = SafetyEvent()
        evt.header.stamp    = self.get_clock().now().to_msg()
        evt.header.frame_id = self.ns
        evt.drone_id    = self.drone_id
        evt.event_type  = 'BATTERY_CRITICAL'
        evt.description = (
            f'drone{self.drone_id} batarya kritik: '
            f'%{pct:.1f} < eşik %{BATTERY_CRITICAL_PCT}'
        )
        evt.severity = min(1.0, (BATTERY_CRITICAL_PCT - pct) / BATTERY_CRITICAL_PCT + 0.7)
        self._safety_pub.publish(evt)

        self.get_logger().error(
            f'🔴 [{self.ns}] BATTERY_CRITICAL! %{pct:.1f} → SAFETY_HOLD'
        )

    # ══════════════════════════════════════════════════════════════════════════
    # GEOFENCE KONTROLÜ
    # ══════════════════════════════════════════════════════════════════════════

    def _check_geofence(self, now: float) -> None:
        if self._own_pose is None:
            return

        # Stale pose → geofence kontrolü yapma
        if now - self._own_pose_time > 2.0:
            return

        x, y, z = self._own_pose

        # Sınır ihlali tespiti
        breach = (
            x < FENCE_X_MIN or x > FENCE_X_MAX or
            y < FENCE_Y_MIN or y > FENCE_Y_MAX or
            z > FENCE_ALT_MAX or z < FENCE_ALT_MIN
        )

        # Yaklaşma uyarısı (sınırdan FENCE_MARGIN_M önce, 10s'de bir)
        near_fence = (
            x < FENCE_X_MIN + FENCE_MARGIN_M or
            x > FENCE_X_MAX - FENCE_MARGIN_M or
            y < FENCE_Y_MIN + FENCE_MARGIN_M or
            y > FENCE_Y_MAX - FENCE_MARGIN_M or
            z > FENCE_ALT_MAX - FENCE_MARGIN_M
        )
        if near_fence and not breach:
            if now - self._last_geofence_warn_log > 10.0:
                self._last_geofence_warn_log = now
                self.get_logger().warn(
                    f'⚠  [{self.ns}] Geofence sınırına yakın: '
                    f'({x:.1f}, {y:.1f}, {z:.1f})'
                )

        if not breach:
            return

        # Cooldown kontrol
        if now - self._last_geofence_event_time < FENCE_EVENT_COOLDOWN_S:
            return

        self._last_geofence_event_time = now

        # Hangi sınır ihlal edildi?
        details = []
        if x < FENCE_X_MIN:  details.append(f'x={x:.1f}<{FENCE_X_MIN}')
        if x > FENCE_X_MAX:  details.append(f'x={x:.1f}>{FENCE_X_MAX}')
        if y < FENCE_Y_MIN:  details.append(f'y={y:.1f}<{FENCE_Y_MIN}')
        if y > FENCE_Y_MAX:  details.append(f'y={y:.1f}>{FENCE_Y_MAX}')
        if z > FENCE_ALT_MAX: details.append(f'z={z:.1f}>{FENCE_ALT_MAX}')
        if z < FENCE_ALT_MIN: details.append(f'z={z:.1f}<{FENCE_ALT_MIN}')

        # Merkezden uzaklık (severity için)
        cx = (FENCE_X_MAX + FENCE_X_MIN) / 2
        cy = (FENCE_Y_MAX + FENCE_Y_MIN) / 2
        d_center = math.sqrt((x - cx)**2 + (y - cy)**2)
        d_max    = math.sqrt(((FENCE_X_MAX - FENCE_X_MIN)/2)**2 +
                              ((FENCE_Y_MAX - FENCE_Y_MIN)/2)**2)
        severity = min(1.0, d_center / d_max)

        evt = SafetyEvent()
        evt.header.stamp    = self.get_clock().now().to_msg()
        evt.header.frame_id = self.ns
        evt.drone_id    = self.drone_id
        evt.event_type  = 'GEOFENCE_BREACH'
        evt.description = (
            f'drone{self.drone_id} geofence ihlali: '
            f'{", ".join(details)}'
        )
        evt.severity = severity
        self._safety_pub.publish(evt)

        self.get_logger().error(
            f'🔴 [{self.ns}] GEOFENCE_BREACH! {", ".join(details)} → SAFETY_HOLD'
        )

    # ══════════════════════════════════════════════════════════════════════════
    # HEARTBEAT KONTROLÜ
    # ══════════════════════════════════════════════════════════════════════════

    def _check_heartbeat(self, now: float) -> None:
        """intent_coordinator'dan mesaj geliyor mu?"""
        if self._last_intent_time == 0.0:
            return  # Henüz hiç intent gelmedi — normal (görev başlamadı)

        age = now - self._last_intent_time
        if age > HEARTBEAT_TIMEOUT_S:
            if now - self._last_heartbeat_warn_log > 5.0:
                self._last_heartbeat_warn_log = now
                self.get_logger().warn(
                    f'💔 [{self.ns}] Intent heartbeat kesildi: '
                    f'{age:.1f}s sessizlik (eşik={HEARTBEAT_TIMEOUT_S}s)'
                )
                # Not: GNSS_DEGRADED veya bağlantı kesintisi lokal kararı
                # local_fsm'in kendi INTENT_TIMEOUT_S mekanizması devreye girer.
                # safety_monitor sadece uyarı loglar.

    # ══════════════════════════════════════════════════════════════════════════
    # BATARYA LOG
    # ══════════════════════════════════════════════════════════════════════════

    def _battery_log_tick(self) -> None:
        """10s'de bir batarya durumu logla."""
        self.get_logger().info(
            f'⚡ [{self.ns}] Batarya: %{self._battery_pct:.1f}'
        )


# ══════════════════════════════════════════════════════════════════════════════
# ENTRYPOINT
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
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
# Terminal 1 — safety_monitor (drone1):
#   DRONE_NS=drone1 DRONE_ID=1 \
#   ros2 run my_swarm_pkg safety_monitor
#
# Terminal 2 — geofence ihlali testi (drone1 sınır dışına çıksın):
#   ros2 topic pub /drone1/pose geometry_msgs/msg/PoseStamped \
#     '{pose: {position: {x: 130.0, y: 50.0, z: 20.0}}}' --once
#   # Beklenen: GEOFENCE_BREACH eventi → /safety/event
#
# Terminal 3 — safety event izle:
#   ros2 topic echo /safety/event
#
# Terminal 4 — batarya izle:
#   ros2 topic echo /drone1/battery_pct
#
# Hızlı batarya tükenmesi testi (drain hızı değiştir):
#   DRONE_NS=drone1 DRONE_ID=1 BATTERY_DRAIN_PER_S=0.1 \
#   ros2 run my_swarm_pkg safety_monitor
#   # %15 altına düşme süresi: ~850s → 85s
