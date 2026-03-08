#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                         qr_perception.py                                    ║
║         Mock QR Algılama Nodu — Şartname Şekil 2 JSON Formatı               ║
╚══════════════════════════════════════════════════════════════════════════════╝

GENEL AÇIKLAMA:
---------------
Gerçek kamera olmadan QR algılamayı simüle eder.
Kamera drone'u bir QR'a trigger_radius kadar yaklaşınca YAML'daki
'content' alanı okunur (Şekil 2 JSON formatı ile birebir) ve QRResult yayınlanır.

Şekil 2 JSON yapısı:
  {
    "qr_id": 1,
    "gorev": {
      "formasyon":          {"aktif": true,  "tip": "OKBASI"},
      "manevra_pitch_roll": {"aktif": false, "pitch_deg": "0", "roll_deg": "0"},
      "irtifa_degisim":     {"aktif": true,  "deger": 20},
      "bekleme_suresi_s":   3
    },
    "suruden_ayrilma": {
      "aktif": false,
      "ayrilacak_drone_id": null,
      "hedef_renk": null,
      "bekleme_suresi_s": null
    },
    "sonraki_qr": {"team_1": 4, "team_2": 4, "team_3": 4}
  }

TEAM ID HARİTALAMA:
-------------------
  ROS parametre 'team_id' (örn: 'team1') → YAML anahtar 'team_1'
  Dönüşüm: team_id.replace('team', 'team_')
  Örn: 'team1' → 'team_1', 'team2' → 'team_2', 'team3' → 'team_3'

VERİ AKIŞI:
-----------
  /drone{i}/pose       →  10Hz yakınlık kontrolü
  /drone{i}/local_state →  failover izleme
                                ↓
                       qr_perception (BU NODE)
                          ↓              ↓
                   /qr/result      /perception/color_zones
                          ↓
                  intent_coordinator

BAŞLATMA:
---------
  ros2 run my_swarm_pkg qr_perception --ros-args -p team_id:=team1

TEST:
-----
  ros2 topic pub /drone1/pose geometry_msgs/msg/PoseStamped \
      "{pose: {position: {x: 10.0, y: 20.0, z: 15.0}}}" --once
  ros2 topic echo /qr/result
"""

import os
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from swarm_msgs.msg import LocalState, QRResult, ColorZone, ColorZoneList

try:
    from ament_index_python.packages import get_package_share_directory
    _AMENT_AVAILABLE = True
except ImportError:
    _AMENT_AVAILABLE = False

# Kamera failover'ı tetikleyen yerel durumlar
_FAILOVER_STATES = frozenset({'DETACH', 'SAFETY_HOLD', 'PILOT_OVERRIDE'})
_RECOVERY_STATE  = 'REJOIN'


class QRPerception(Node):
    """Mock QR algılama nodu — Şekil 2 JSON formatı."""

    def __init__(self):
        super().__init__('qr_perception')

        # ── PARAMETRELER ─────────────────────────────────────────────────
        self.declare_parameter('team_id',                'team1')
        self.declare_parameter('camera_drone_id',        1)
        self.declare_parameter('backup_camera_drone_id', 2)
        self.declare_parameter('num_drones',             3)
        self.declare_parameter('trigger_radius',         5.0)
        self.declare_parameter('qr_map_file',            '')

        self._team_id        = self.get_parameter('team_id').value
        self._primary_cam_id = self.get_parameter('camera_drone_id').value
        self._backup_cam_id  = self.get_parameter('backup_camera_drone_id').value
        self._num_drones     = self.get_parameter('num_drones').value
        self._trigger_radius = self.get_parameter('trigger_radius').value
        qr_map_file          = self.get_parameter('qr_map_file').value

        # 'team1' → 'team_1' (YAML anahtar uyumu)
        self._team_key = self._team_id.replace('team', 'team_')

        # Aktif kamera drone
        self._camera_drone_id: int  = self._primary_cam_id
        self._failover_active: bool = False

        # ── QR HARİTASI ──────────────────────────────────────────────────
        if not qr_map_file:
            if _AMENT_AVAILABLE:
                pkg = get_package_share_directory('my_swarm_pkg')
                qr_map_file = os.path.join(pkg, 'config', 'qr_map.yaml')
            else:
                qr_map_file = os.path.join(
                    os.path.dirname(__file__), '..', '..', 'config', 'qr_map.yaml')

        self._qr_nodes:   dict = {}
        self._color_zones: list = []
        self._load_qr_map(qr_map_file)

        # ── DURUM ────────────────────────────────────────────────────────
        self._drone_poses:  dict[int, PoseStamped | None] = {
            i: None for i in range(1, self._num_drones + 1)
        }
        self._drone_states: dict[int, str] = {
            i: 'STANDBY' for i in range(1, self._num_drones + 1)
        }
        self._read_qr_ids: set[int] = set()   # aynı QR iki kez okunmaz

        # ── QoS ─────────────────────────────────────────────────────────
        be_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── PUBLISHER'LAR ────────────────────────────────────────────────
        self._qr_pub   = self.create_publisher(QRResult,      '/qr/result',               10)
        self._zone_pub = self.create_publisher(ColorZoneList, '/perception/color_zones',  10)

        # ── ABONELİKLER ─────────────────────────────────────────────────
        for i in range(1, self._num_drones + 1):
            self.create_subscription(
                PoseStamped, f'/drone{i}/pose',
                lambda msg, did=i: self._on_pose(msg, did), be_qos)
            self.create_subscription(
                LocalState, f'/drone{i}/local_state',
                lambda msg, did=i: self._on_local_state(msg, did), 10)

        # ── TIMER'LAR ────────────────────────────────────────────────────
        self.create_timer(0.1, self._check_qr_proximity)   # 10 Hz
        self.create_timer(1.0, self._publish_color_zones)  # 1 Hz

        self.get_logger().info(
            f'\n📷 qr_perception hazır!\n'
            f'   Takım ID     : {self._team_id}  (YAML key: {self._team_key})\n'
            f'   Kamera drone : drone{self._camera_drone_id}\n'
            f'   Yedek        : drone{self._backup_cam_id}\n'
            f'   QR düğüm     : {len(self._qr_nodes)}\n'
            f'   Renk zone    : {len(self._color_zones)}\n'
            f'   Trig. radius : {self._trigger_radius:.1f} m\n'
            f'   Rota         : '
            + ' → '.join(str(k) for k in sorted(self._qr_nodes.keys()))
        )

    # ── YAML YÜKLEME ─────────────────────────────────────────────────────

    def _load_qr_map(self, filepath: str):
        """YAML'dan QR haritasını yükle (Şekil 2 formatı)."""
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)

            self._qr_nodes    = data.get('qr_nodes',    {})
            self._color_zones = data.get('color_zones', [])

            if 'trigger_radius' in data:
                self._trigger_radius = float(data['trigger_radius'])

            # Doğrulama
            for qr_id, node in self._qr_nodes.items():
                if 'position' not in node:
                    self.get_logger().warn(f'⚠️  QR{qr_id}: position eksik!')
                if 'content' not in node:
                    self.get_logger().warn(f'⚠️  QR{qr_id}: content eksik!')
                else:
                    c = node['content']
                    if 'sonraki_qr' not in c:
                        self.get_logger().warn(f'⚠️  QR{qr_id}: content.sonraki_qr eksik!')
                    elif self._team_key not in c['sonraki_qr']:
                        self.get_logger().warn(
                            f'⚠️  QR{qr_id}: sonraki_qr["{self._team_key}"] eksik!')

            self.get_logger().info(f'📂 QR haritası yüklendi: {filepath}')

        except FileNotFoundError:
            self.get_logger().error(f'❌ Dosya bulunamadı: {filepath}')
        except yaml.YAMLError as e:
            self.get_logger().error(f'❌ YAML parse hatası: {e}')
        except Exception as e:
            self.get_logger().error(f'❌ Yükleme hatası: {e}')

    # ── CALLBACK'LER ─────────────────────────────────────────────────────

    def _on_pose(self, msg: PoseStamped, drone_id: int):
        self._drone_poses[drone_id] = msg

    def _on_local_state(self, msg: LocalState, drone_id: int):
        """Failover: primary kamera drone tehlikedeyse yedek devralır."""
        old_state = self._drone_states.get(drone_id, 'STANDBY')
        self._drone_states[drone_id] = msg.state

        if (drone_id == self._primary_cam_id
                and old_state not in _FAILOVER_STATES
                and msg.state in _FAILOVER_STATES
                and not self._failover_active):
            self._failover_active = True
            self._camera_drone_id = self._backup_cam_id
            self.get_logger().warn(
                f'⚠️  KAMERA FAILOVER: drone{self._primary_cam_id} → {msg.state}\n'
                f'   Yedek aktif: drone{self._backup_cam_id}')

        if (drone_id == self._primary_cam_id
                and self._failover_active
                and msg.state == _RECOVERY_STATE):
            self._failover_active = False
            self._camera_drone_id = self._primary_cam_id
            self.get_logger().info(
                f'✅ Kamera primary\'e döndü: drone{self._primary_cam_id}')

    # ── QR YAKIŞLIK KONTROLÜ (10 Hz) ────────────────────────────────────

    def _check_qr_proximity(self):
        """Kamera drone'u QR'a yeterince yaklaştı mı? → QRResult yayınla."""
        cam_pose = self._drone_poses.get(self._camera_drone_id)
        if cam_pose is None:
            return

        cx = cam_pose.pose.position.x
        cy = cam_pose.pose.position.y

        for qr_id, qr_data in self._qr_nodes.items():
            if qr_id in self._read_qr_ids:
                continue

            qpos = qr_data.get('position', {})
            dx   = cx - float(qpos.get('x', 0.0))
            dy   = cy - float(qpos.get('y', 0.0))
            dist = math.sqrt(dx * dx + dy * dy)

            if dist <= self._trigger_radius:
                self.get_logger().info(
                    f'📍 QR{qr_id} menzilinde! mesafe={dist:.2f}m')
                self._publish_qr_result(qr_id, qr_data)
                self._read_qr_ids.add(qr_id)

    # ── QR SONUCU YAYINLAMA ──────────────────────────────────────────────

    def _publish_qr_result(self, qr_id: int, qr_data: dict):
        """
        Şekil 2 JSON yapısını parse et → QRResult mesajı yayınla.

        YAML content:
          gorev.formasyon          → formation_active, formation_type
          gorev.manevra_pitch_roll → maneuver_active, pitch_deg, roll_deg
          gorev.irtifa_degisim     → altitude_active, altitude
          gorev.bekleme_suresi_s   → wait_seconds
          suruden_ayrilma          → detach_active, detach_drone_id, zone_color,
                                     detach_wait_seconds
          sonraki_qr[team_key]     → next_qr_id
        """
        content = qr_data.get('content', {})
        if not content:
            self.get_logger().warn(f'[QR{qr_id}] content alanı boş → DROP')
            return

        # ── sonraki_qr ────────────────────────────────────────────────
        sonraki_qr = content.get('sonraki_qr', {})
        if self._team_key not in sonraki_qr:
            self.get_logger().warn(
                f'[QR{qr_id}] sonraki_qr["{self._team_key}"] yok → DROP')
            return
        next_qr_id = int(sonraki_qr[self._team_key])

        # ── gorev alanları ────────────────────────────────────────────
        gorev     = content.get('gorev', {})
        frm       = gorev.get('formasyon',          {})
        maneuver  = gorev.get('manevra_pitch_roll',  {})
        irtifa    = gorev.get('irtifa_degisim',      {})
        bekleme   = gorev.get('bekleme_suresi_s',     3)

        # ── suruden_ayrilma ──────────────────────────────────────────
        ayrilma   = content.get('suruden_ayrilma', {})

        # ── MESAJ ────────────────────────────────────────────────────
        msg = QRResult()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.team_id = self._team_id
        msg.qr_id   = int(qr_id)

        # Formasyon
        msg.formation_active = bool(frm.get('aktif', False))
        msg.formation_type   = str(frm.get('tip',   'OKBASI'))
        msg.drone_spacing    = 6.0   # sabit — şartname §5.2.1

        # İrtifa
        msg.altitude_active  = bool(irtifa.get('aktif', False))
        msg.altitude         = float(irtifa.get('deger', 15.0))

        # Manevra
        msg.maneuver_active  = bool(maneuver.get('aktif', False))
        msg.pitch_deg        = float(maneuver.get('pitch_deg', '0'))
        msg.roll_deg         = float(maneuver.get('roll_deg',  '0'))

        # Sürüden ayrılma
        msg.detach_active   = bool(ayrilma.get('aktif', False))
        msg.detach_drone_id = int(ayrilma.get('ayrilacak_drone_id') or 0)
        msg.zone_color      = str(ayrilma.get('hedef_renk') or '')

        # Zamanlama
        msg.next_qr_id   = next_qr_id
        msg.wait_seconds = float(bekleme)

        # Bu QR'ın konumu
        qpos = qr_data.get('position', {})
        msg.qr_position.x = float(qpos.get('x', 0.0))
        msg.qr_position.y = float(qpos.get('y', 0.0))
        msg.qr_position.z = float(qpos.get('z', 0.0))

        # Sonraki QR konumu (next_qr_id=0 → HOME)
        if next_qr_id != 0 and next_qr_id in self._qr_nodes:
            npos = self._qr_nodes[next_qr_id].get('position', {})
            msg.next_qr_position.x = float(npos.get('x', 0.0))
            msg.next_qr_position.y = float(npos.get('y', 0.0))
            msg.next_qr_position.z = float(npos.get('z', 0.0))

        self._qr_pub.publish(msg)

        # ── LOG ──────────────────────────────────────────────────────
        ayrilma_str = (f'drone{msg.detach_drone_id}→{msg.zone_color}'
                       if msg.detach_active else '—')
        home_str    = ' ← SON QR → 🏠 HOME!' if next_qr_id == 0 else ''
        self.get_logger().info(
            f'\n╔══ 📷 QR{qr_id} OKUNDU [drone{self._camera_drone_id}] ══\n'
            f'║  Formasyon  : {msg.formation_type} '
            f'[{"✅" if msg.formation_active else "—"}]\n'
            f'║  İrtifa     : {msg.altitude}m '
            f'[{"✅" if msg.altitude_active else "—"}]\n'
            f'║  Manevra    : pitch={msg.pitch_deg}° roll={msg.roll_deg}° '
            f'[{"✅" if msg.maneuver_active else "—"}]\n'
            f'║  Ayırma     : {ayrilma_str} '
            f'[{"✅" if msg.detach_active else "—"}]\n'
            f'║  Sonraki QR : {next_qr_id}{home_str}\n'
            f'║  Bekle      : {msg.wait_seconds}s\n'
            f'╚{"═"*40}'
        )

    # ── RENK ZONE YAYINI (1 Hz) ──────────────────────────────────────────

    def _publish_color_zones(self):
        """1 Hz: tüm renk bölgelerini /perception/color_zones'a yayınla."""
        if not self._color_zones:
            return

        msg = ColorZoneList()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for z in self._color_zones:
            zone           = ColorZone()
            zone.position.x = float(z.get('x',      0.0))
            zone.position.y = float(z.get('y',      0.0))
            zone.position.z = float(z.get('z',      0.0))
            zone.color      = str(z.get('color',   'RED'))
            zone.radius     = float(z.get('radius',  3.0))
            msg.zones.append(zone)

        self._zone_pub.publish(msg)

    # ── YARDIMCI ─────────────────────────────────────────────────────────

    def reset_qr_memory(self):
        """Okunan QR'ları sıfırla (mission_fsm RESET komutu için)."""
        self._read_qr_ids.clear()
        self.get_logger().info('🔄 QR belleği sıfırlandı.')

    def get_unread_qr_ids(self) -> list[int]:
        """Henüz okunmamış QR ID'lerini döndür (test)."""
        return [q for q in self._qr_nodes if q not in self._read_qr_ids]


# ── MAIN ──────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = QRPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⛔ qr_perception durduruluyor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
