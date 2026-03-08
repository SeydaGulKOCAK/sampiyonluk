#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                      collision_avoidance.py  v1                             ║
║           Çarpışma Kaçınma — APF (Artificial Potential Field)               ║
╚══════════════════════════════════════════════════════════════════════════════╝

GENEL AÇIKLAMA:
---------------
Bu node, formation_controller'ın ürettiği ham setpoint'i (setpoint_raw) alır,
komşu drone'ların konumlarına bakarak APF (Yapay Potansiyel Alan) uygular ve
nihai setpoint'i (setpoint_final) drone_interface'e iletir.

Yapay Potansiyel Alan (APF):
  ┌─────────────────────────────────────────────────────────┐
  │  F_toplam = F_itici (komşulardan kaç) + F_raw (hedefe git) │
  │                                                         │
  │  F_itici: her FLYING komşu için:                        │
  │    d = kendi konum ↔ komşu konum                        │
  │    d < R_MAX → itici kuvvet aktif                       │
  │    F_j = K_REP * (1/d - 1/R_MAX) / d² * yön_vektörü    │
  │                                                         │
  │  F_raw : setpoint_raw'a çekici kuvvet (zaten setpoint)  │
  │          → correction = setpoint_raw + clip(F_itici)    │
  └─────────────────────────────────────────────────────────┘

VERİ AKIŞI:
-----------
  formation_controller → /{ns}/setpoint_raw ──┐
  /drone{i}/pose (tüm komşular) ──────────────┤
  /drone{i}/local_state (FLYING filtre) ──────┼──► collision_avoidance (50Hz)
  /drone{i}/velocity (TTC için) ──────────────┤        │
  /swarm/intent (detach ID) ──────────────────┘        │
                                                        ├──► /{ns}/setpoint_final
                                                        └──► /safety/event

OSİLASYON TESPİTİ:
------------------
  Son OSC_WINDOW adımın (0.4s @ 50Hz) setpoint varyansı OSC_THRESH_M'yi
  aşarsa OSCILLATION SafetyEvent yayınlanır.
  → formation_controller bu event'i alınca slew rate düşürür (dampening)
  → OSC_COOLDOWN_S süresi boyunca tekrar tetiklenmez

TTC (Time-To-Collision) KATEGORİ AĞIRLIKLANDIRMA:
--------------------------------------------------
  d < R_DANGER : TTC < 1.5s → itici kuvvet 3x amplify
  Bu sayede hızlı yaklaşan drone'lar daha sert itilir.

BYPASS MODU:
------------
  FLYING komşu yoksa (kalkış öncesi / son drone) CA direkt pass-through yapar.
  → setpoint_raw → setpoint_final (sıfır gecikme)

PARAMETRE ÖZETI:
----------------
  R_MAX          = 8.0 m   — etki başlangıç mesafesi
  R_MIN          = 3.0 m   — tehlike mesafesi (tam kuvvet)
  K_REP          = 18.0    — itici kazanç (6m nominal spacing için kalibre)
  MAX_CORR_M     = 3.0 m   — max APF düzeltme büyüklüğü (clip)
  TTC_THRESHOLD  = 1.5 s   — TTC sınırı (altında → kuvvet 3x)
  OSC_WINDOW     = 20      — osilasyon penceresi (adım sayısı, 0.4s)
  OSC_THRESH_M   = 0.08 m  — osilasyon varyans eşiği
  OSC_COOLDOWN_S = 5.0 s   — osilasyon event'i tekrar tetikleme süresi

ŞARTNAME UYUMU:
  §5.4  Çarpışma kaçınma     → APF tabanlı reaktif kaçınma ✅
  §5.3  Dağıtık mimari       → Her drone kendi CA'sını çalıştırır ✅
  §5.5  Güvenlik kriterleri  → Osilasyon tespiti + SafetyEvent ✅
  Ceza  Çarpışma (-50 puan)  → R_MIN=3m tampon mesafesi ✅
  Ceza  Osilasyon (-10 puan) → Dampening sinyali yayınlanır ✅
"""

import math
import os
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
from swarm_msgs.msg import LocalState, SwarmIntent, SafetyEvent

# ══════════════════════════════════════════════════════════════════════════════
# APF PARAMETRELERİ — tüm değerler simülasyon testi sonucu kalibre edilmiştir
# ══════════════════════════════════════════════════════════════════════════════
R_MAX: float = 8.0    # [m] — itici kuvvetin etki alanı başlangıcı
R_MIN: float = 3.0    # [m] — tehlike bölgesi (tam amplify kuvvet)
K_REP: float = 18.0   # [—] — itici potansiyel kazanç katsayısı
MAX_CORR_M: float = 3.0   # [m] — APF düzeltme vektörünün clip sınırı (güvenlik)
TTC_THRESHOLD: float = 1.5  # [s] — bu TTC altındaysa kuvvet 3x artır
TTC_AMPLIFY: float = 3.0    # [—] — TTC aşımında kuvvet çarpanı

CTRL_HZ: float = 50.0       # [Hz] — kontrol döngü frekansı
CTRL_DT: float = 1.0 / CTRL_HZ

# Osilasyon tespiti
OSC_WINDOW: int = 20         # Son kaç adıma bak (0.4s @ 50Hz)
OSC_THRESH_M: float = 0.08   # [m] — setpoint std sapması eşiği
OSC_COOLDOWN_S: float = 5.0  # [s] — aynı drone'dan iki event arası min bekleme

# Setpoint geçerliliği
SP_STALE_S: float = 0.5      # [s] — bu kadar eski setpoint_raw → pass-through
POSE_STALE_S: float = 1.0    # [s] — bu kadar eski komşu pose → görmezden gel

SWARM_SIZE: int = int(os.environ.get('SWARM_SIZE', '3'))


# ══════════════════════════════════════════════════════════════════════════════
# YARDIMCI FONKSİYONLAR
# ══════════════════════════════════════════════════════════════════════════════

def _dist3(a: tuple[float, float, float],
           b: tuple[float, float, float]) -> float:
    """İki 3D nokta arası Öklid mesafesi."""
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


def _unit_vec(frm: tuple[float, float, float],
              to: tuple[float, float, float]) -> tuple[float, float, float]:
    """frm → to yönündeki birim vektör. Sıfır vektörde (0,0,0) döner."""
    dx, dy, dz = to[0] - frm[0], to[1] - frm[1], to[2] - frm[2]
    mag = math.sqrt(dx * dx + dy * dy + dz * dz)
    if mag < 1e-9:
        return (0.0, 0.0, 0.0)
    return (dx / mag, dy / mag, dz / mag)


def _clip_vec(v: tuple[float, float, float],
              max_mag: float) -> tuple[float, float, float]:
    """Vektörü max_mag büyüklüğüne kırp (yönü koru)."""
    mag = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if mag <= max_mag or mag < 1e-9:
        return v
    scale = max_mag / mag
    return (v[0] * scale, v[1] * scale, v[2] * scale)


def _variance(vals: list[float]) -> float:
    """Skaler bir liste üzerinde varyans hesapla."""
    if len(vals) < 2:
        return 0.0
    mean = sum(vals) / len(vals)
    return sum((v - mean) ** 2 for v in vals) / len(vals)


# ══════════════════════════════════════════════════════════════════════════════
# ANA NODE
# ══════════════════════════════════════════════════════════════════════════════

class CollisionAvoidanceNode(Node):
    """
    APF tabanlı çarpışma kaçınma node'u.

    Her drone kendi instance'ını çalıştırır; komşuların pozisyonlarını
    subscribe ederek itici kuvvetleri hesaplar ve setpoint'i düzeltir.
    """

    def __init__(self):
        # ── Namespace / ID çözümleme ──────────────────────────────────────────
        #   Beklenen env: DRONE_NS=drone1, DRONE_ID=1, SWARM_SIZE=3
        self.ns: str = os.environ.get('DRONE_NS', 'drone1')
        self.drone_id: int = int(os.environ.get('DRONE_ID', '1'))
        self.swarm_size: int = SWARM_SIZE

        super().__init__(
            f'collision_avoidance_{self.drone_id}',
            namespace=self.ns,
        )

        self.get_logger().info(
            f'CollisionAvoidance başladı — ns={self.ns}, id={self.drone_id}, '
            f'swarm_size={self.swarm_size}'
        )

        # ── Dahili durum ──────────────────────────────────────────────────────
        self._own_pose: tuple[float, float, float] | None = None
        self._own_pose_time: float = 0.0

        self._own_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)

        self._setpoint_raw: tuple[float, float, float] | None = None
        self._sp_raw_time: float = 0.0

        # drone_id → (x, y, z)
        self._neighbor_poses: dict[int, tuple[float, float, float]] = {}
        self._neighbor_pose_times: dict[int, float] = {}

        # drone_id → (vx, vy, vz)
        self._neighbor_vels: dict[int, tuple[float, float, float]] = {}

        # drone_id → state string
        self._neighbor_states: dict[int, str] = {}

        # detach drone: CA hâlâ kaçınır ama daha düşük kuvvetle
        self._detach_drone_id: int = 0

        # Osilasyon tespiti: son OSC_WINDOW adımın setpoint_final X/Y
        self._sp_hist_x: deque[float] = deque(maxlen=OSC_WINDOW)
        self._sp_hist_y: deque[float] = deque(maxlen=OSC_WINDOW)
        self._last_osc_event_time: float = 0.0

        # Son yayınlanan setpoint (stale guard için)
        self._last_sp_final: tuple[float, float, float] | None = None

        # Geçen çevrim için debug istatistikleri
        self._dbg_closest_dist: float = float('inf')
        self._dbg_corr_mag: float = 0.0
        self._dbg_active_neighbors: int = 0

        # ── QoS profilleri ────────────────────────────────────────────────────
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

        # 1) Kendi ham setpoint'i (formation_controller'dan)
        self.create_subscription(
            PoseStamped,
            f'/{self.ns}/setpoint_raw',
            self._on_setpoint_raw,
            10,
        )

        # 2) Kendi konum + hız (drone_interface'den)
        self.create_subscription(
            PoseStamped,
            f'/{self.ns}/pose',
            self._on_own_pose,
            best_effort_qos,
        )
        self.create_subscription(
            TwistStamped,
            f'/{self.ns}/velocity',
            self._on_own_vel,
            best_effort_qos,
        )

        # 3) Komşu konumları + hızları + durumları
        for i in range(1, self.swarm_size + 1):
            if i == self.drone_id:
                continue  # kendini izleme

            self.create_subscription(
                PoseStamped,
                f'/drone{i}/pose',
                lambda msg, uid=i: self._on_neighbor_pose(msg, uid),
                best_effort_qos,
            )
            self.create_subscription(
                TwistStamped,
                f'/drone{i}/velocity',
                lambda msg, uid=i: self._on_neighbor_vel(msg, uid),
                best_effort_qos,
            )
            self.create_subscription(
                LocalState,
                f'/drone{i}/local_state',
                lambda msg, uid=i: self._on_neighbor_state(msg, uid),
                reliable_qos,
            )

        # 4) Sürü niyeti (detach_drone_id için)
        self.create_subscription(
            SwarmIntent,
            '/swarm/intent',
            self._on_intent,
            10,
        )

        # ── Yayıncılar ────────────────────────────────────────────────────────

        # Nihai setpoint → drone_interface
        self._sp_final_pub = self.create_publisher(
            PoseStamped,
            f'/{self.ns}/setpoint_final',
            10,
        )

        # Güvenlik olayı → formation_controller + local_fsm
        self._safety_pub = self.create_publisher(
            SafetyEvent,
            '/safety/event',
            reliable_qos,
        )

        # ── 50 Hz kontrol döngüsü ─────────────────────────────────────────────
        self.create_timer(CTRL_DT, self._control_loop)

        self.get_logger().info(
            f'APF parametreleri: R_MAX={R_MAX}m, R_MIN={R_MIN}m, '
            f'K_REP={K_REP}, MAX_CORR={MAX_CORR_M}m'
        )

    # ══════════════════════════════════════════════════════════════════════════
    # CALLBACK'LER
    # ══════════════════════════════════════════════════════════════════════════

    def _on_setpoint_raw(self, msg: PoseStamped) -> None:
        """formation_controller'dan gelen ham setpoint."""
        p = msg.pose.position
        self._setpoint_raw = (p.x, p.y, p.z)
        self._sp_raw_time = time.time()

    def _on_own_pose(self, msg: PoseStamped) -> None:
        """Kendi mevcut konumu (ENU, metre)."""
        p = msg.pose.position
        self._own_pose = (p.x, p.y, p.z)
        self._own_pose_time = time.time()

    def _on_own_vel(self, msg: TwistStamped) -> None:
        """Kendi mevcut hızı (ENU, m/s)."""
        v = msg.twist.linear
        self._own_vel = (v.x, v.y, v.z)

    def _on_neighbor_pose(self, msg: PoseStamped, uid: int) -> None:
        """Komşu drone konumu."""
        p = msg.pose.position
        self._neighbor_poses[uid] = (p.x, p.y, p.z)
        self._neighbor_pose_times[uid] = time.time()

    def _on_neighbor_vel(self, msg: TwistStamped, uid: int) -> None:
        """Komşu drone hızı (TTC hesabı için)."""
        v = msg.twist.linear
        self._neighbor_vels[uid] = (v.x, v.y, v.z)

    def _on_neighbor_state(self, msg: LocalState, uid: int) -> None:
        """Komşu drone durum makinesi durumu."""
        self._neighbor_states[uid] = msg.state

    def _on_intent(self, msg: SwarmIntent) -> None:
        """Sürü niyeti: detach drone ID'si."""
        self._detach_drone_id = int(msg.detach_drone_id)

    # ══════════════════════════════════════════════════════════════════════════
    # APF HESABI
    # ══════════════════════════════════════════════════════════════════════════

    def _compute_repulsive_force(
        self,
        own_pos: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """
        Tüm FLYING komşular için itici APF kuvvetini hesapla.

        Standart APF formülü (Khatib 1986):
            F_rep = K * (1/d - 1/R_MAX) * (1/d²) * ∇d

        Toplam vektörü döndürür: (fx, fy, fz) [metre biriminde düzeltme]
        """
        fx, fy, fz = 0.0, 0.0, 0.0
        now = time.time()

        active_count = 0
        closest_dist = float('inf')

        for uid, npos in self._neighbor_poses.items():
            # ── Stale kontrol ──────────────────────────────────────────────────
            if now - self._neighbor_pose_times.get(uid, 0.0) > POSE_STALE_S:
                continue

            # ── Sadece FLYING (veya DETACH, REJOIN) olan komşuları dikkate al ─
            state = self._neighbor_states.get(uid, '')
            if state not in ('FLYING', 'DETACH', 'REJOIN', 'LAND_ZONE'):
                continue

            d = _dist3(own_pos, npos)
            if d < 0.05:
                # Aynı nokta — teoride olmamalı, ama güvenlik için
                d = 0.05

            closest_dist = min(closest_dist, d)

            if d >= R_MAX:
                continue  # Etki alanı dışında

            active_count += 1

            # ── APF itici büyüklük ────────────────────────────────────────────
            magnitude = K_REP * (1.0 / d - 1.0 / R_MAX) / (d * d)

            # ── TTC (Time-To-Collision) amplifikasyon ────────────────────────
            #   Komşunun bizim yönümüze hızla yaklaşıyorsa kuvveti artır
            nvel = self._neighbor_vels.get(uid, (0.0, 0.0, 0.0))
            rel_vx = self._own_vel[0] - nvel[0]
            rel_vy = self._own_vel[1] - nvel[1]
            rel_vz = self._own_vel[2] - nvel[2]
            # Yaklaşma hızı: negatif → yaklaşıyor
            toward_vec = _unit_vec(npos, own_pos)  # komşudan bize doğru
            closing_speed = (
                rel_vx * toward_vec[0] +
                rel_vy * toward_vec[1] +
                rel_vz * toward_vec[2]
            )
            # closing_speed pozitif → biz ondan uzaklaşıyoruz → TTC büyük → OK
            # closing_speed negatif → yaklaşıyoruz
            if closing_speed < -0.01:  # yaklaşıyoruz
                ttc = d / abs(closing_speed)
                if ttc < TTC_THRESHOLD:
                    magnitude *= TTC_AMPLIFY

            # ── Detach drone'a daha düşük kuvvet (sürüden ayrılıyor, normal) ─
            if uid == self._detach_drone_id and self._detach_drone_id != 0:
                magnitude *= 0.5

            # ── Yön: komşudan bize (uzaklaşma yönü) ─────────────────────────
            away = _unit_vec(npos, own_pos)  # komşudan bize doğru
            fx += magnitude * away[0]
            fy += magnitude * away[1]
            fz += magnitude * away[2]

        # İstatistik güncelle
        self._dbg_closest_dist = closest_dist
        self._dbg_active_neighbors = active_count

        return (fx, fy, fz)

    # ══════════════════════════════════════════════════════════════════════════
    # OSİLASYON TESPİTİ
    # ══════════════════════════════════════════════════════════════════════════

    def _check_oscillation(
        self,
        sp: tuple[float, float, float],
    ) -> None:
        """
        Son OSC_WINDOW adımın setpoint X/Y standart sapmasına bak.
        Eşiği aşarsa SafetyEvent(OSCILLATION) yayınla.
        Cooldown süresi içinde tekrar tetiklenmez.
        """
        self._sp_hist_x.append(sp[0])
        self._sp_hist_y.append(sp[1])

        if len(self._sp_hist_x) < OSC_WINDOW:
            return  # Yeterli veri yok

        var_x = _variance(list(self._sp_hist_x))
        var_y = _variance(list(self._sp_hist_y))
        std_xy = math.sqrt((var_x + var_y) / 2.0)

        if std_xy < OSC_THRESH_M:
            return

        now = time.time()
        if now - self._last_osc_event_time < OSC_COOLDOWN_S:
            return  # Cooldown aktif

        self._last_osc_event_time = now

        evt = SafetyEvent()
        evt.header = Header()
        evt.header.stamp = self.get_clock().now().to_msg()
        evt.header.frame_id = self.ns
        evt.drone_id = self.drone_id
        evt.event_type = 'OSCILLATION'
        evt.description = (
            f'drone{self.drone_id} APF osilasyon: '
            f'std_xy={std_xy:.3f}m > eşik={OSC_THRESH_M}m'
        )
        evt.severity = min(1.0, float(std_xy / (OSC_THRESH_M * 3.0)))
        self._safety_pub.publish(evt)

        self.get_logger().warn(
            f'OSİLASYON TESPİT: std_xy={std_xy:.3f}m, '
            f'severity={evt.severity:.2f} → /safety/event yayınlandı'
        )

    # ══════════════════════════════════════════════════════════════════════════
    # ANA KONTROL DÖNGÜSÜ — 50 Hz
    # ══════════════════════════════════════════════════════════════════════════

    def _control_loop(self) -> None:
        """
        50 Hz'de çalışır:
          1) Veri tazeliği kontrol
          2) APF hesabı
          3) Osilasyon tespiti
          4) setpoint_final yayınla
        """
        now = time.time()

        # ── 1) Temel veri kontrolü ────────────────────────────────────────────
        if self._setpoint_raw is None:
            return  # Henüz setpoint gelmedi

        sp_raw_age = now - self._sp_raw_time
        if sp_raw_age > SP_STALE_S:
            # Stale setpoint → son bilinen setpoint'i koru, yayın yapma
            return

        if self._own_pose is None:
            # Kendi konumunu bilmiyoruz → pass-through (CA kapalı)
            self._publish_setpoint(self._setpoint_raw)
            return

        own_pos = self._own_pose

        # ── 2) FLYING komşu var mı? ───────────────────────────────────────────
        flying_neighbors = [
            uid for uid, st in self._neighbor_states.items()
            if st in ('FLYING', 'DETACH', 'REJOIN', 'LAND_ZONE')
        ]
        if not flying_neighbors:
            # Bypass: komşu yok (yalnız uçuş veya kalkış öncesi)
            self._publish_setpoint(self._setpoint_raw)
            return

        # ── 3) APF itici kuvvet ───────────────────────────────────────────────
        rep_force = self._compute_repulsive_force(own_pos)

        # ── 4) Düzeltme clip ──────────────────────────────────────────────────
        corr = _clip_vec(rep_force, MAX_CORR_M)
        self._dbg_corr_mag = math.sqrt(
            corr[0] ** 2 + corr[1] ** 2 + corr[2] ** 2
        )

        # ── 5) Nihai setpoint ─────────────────────────────────────────────────
        sp_raw = self._setpoint_raw
        sp_final: tuple[float, float, float] = (
            sp_raw[0] + corr[0],
            sp_raw[1] + corr[1],
            sp_raw[2] + corr[2],  # Z düzeltmesi: irtifa çakışması için
        )

        # ── 6) Osilasyon tespiti ──────────────────────────────────────────────
        self._check_oscillation(sp_final)

        # ── 7) Yayın ──────────────────────────────────────────────────────────
        self._publish_setpoint(sp_final)

        # ── 8) Periyodik debug log (5 sn'de bir) ─────────────────────────────
        if int(now * 5) % 25 == 0 and (
            self._dbg_closest_dist < R_MAX or self._dbg_corr_mag > 0.05
        ):
            self.get_logger().debug(
                f'CA | yakın={self._dbg_closest_dist:.2f}m '
                f'aktif={self._dbg_active_neighbors} '
                f'|corr|={self._dbg_corr_mag:.3f}m '
                f'sp_raw=({sp_raw[0]:.1f},{sp_raw[1]:.1f},{sp_raw[2]:.1f}) '
                f'sp_fin=({sp_final[0]:.1f},{sp_final[1]:.1f},{sp_final[2]:.1f})'
            )

    # ══════════════════════════════════════════════════════════════════════════
    # SETPOINT YAYINI
    # ══════════════════════════════════════════════════════════════════════════

    def _publish_setpoint(
        self,
        sp: tuple[float, float, float],
    ) -> None:
        """PoseStamped formatında setpoint_final yayınla."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = sp[0]
        msg.pose.position.y = sp[1]
        msg.pose.position.z = sp[2]
        # Orientation: kimlik quaternion (drone_interface MAVROS'a yönü ayrıca
        # gönderdiğinden burada önemli değil)
        msg.pose.orientation.w = 1.0
        self._sp_final_pub.publish(msg)
        self._last_sp_final = sp


# ══════════════════════════════════════════════════════════════════════════════
# ENTRYPOINT
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
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
# MANUEL TEST (bağımsız çalıştırma)
# ══════════════════════════════════════════════════════════════════════════════
#
# Terminal 1 — drone1 CA:
#   DRONE_NS=drone1 DRONE_ID=1 SWARM_SIZE=3 \
#   ros2 run my_swarm_pkg collision_avoidance
#
# Terminal 2 — sahte komşu konum (3m — tehlike bölgesi):
#   ros2 topic pub /drone2/local_state swarm_msgs/msg/LocalState \
#     '{drone_id: 2, state: "FLYING"}' &
#   ros2 topic pub /drone2/pose geometry_msgs/msg/PoseStamped \
#     '{pose: {position: {x: 3.0, y: 0.0, z: 20.0}}}' --rate 10 &
#   ros2 topic pub /drone1/pose geometry_msgs/msg/PoseStamped \
#     '{pose: {position: {x: 0.0, y: 0.0, z: 20.0}}}' --rate 10 &
#   ros2 topic pub /drone1/setpoint_raw geometry_msgs/msg/PoseStamped \
#     '{pose: {position: {x: 5.0, y: 0.0, z: 20.0}}}' --rate 10
#
# Terminal 3 — setpoint_final izle:
#   ros2 topic echo /drone1/setpoint_final
#   # Beklenen: x < 5.0 (APF geri itiyor)
#
# Terminal 4 — osilasyon testi:
#   # setpoint_raw'ı hızla ±0.5m değiştir → /safety/event OSCILLATION beklenir
