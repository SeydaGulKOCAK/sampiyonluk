#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                         mission_fsm.py  v1                                  ║
║              GCS Görev Durum Makinesi — Yer Kontrol İstasyonu               ║
╚══════════════════════════════════════════════════════════════════════════════╝

GENEL AÇIKLAMA:
---------------
Bu node GCS (laptop) üzerinde çalışır ve operatörün görevi başlatıp
durdurmasına imkân tanır. Tüm İHA'ların durumunu izleyerek terminal
dashboardu gösterir.

GÖREV AKIŞI:
------------
  Operatör "start" → TaskTrigger(TASK1, start=True) → intent_coordinator
  Operatör "abort" → TaskTrigger(TASK1, start=False) → intent_coordinator
  Görev bitince (tüm drone'lar LANDING) → otomatik COMPLETE

GCS DURUM MAKİNESİ:
-------------------
  STANDBY          → Görev başlamadı, operatör komutu bekleniyor
  TASK1_SENT       → Başlatma komutu gönderildi, dronlar FLYING'e geçiyor
  TASK1_ACTIVE     → Görev aktif, QR navigasyon sürüyor
  ABORT_SENT       → Durdurma komutu gönderildi, RTL bekleniyor
  COMPLETE         → Tüm dronlar indi, görev bitti

OPERATÖR ARAYÜZÜ (terminal):
-----------------------------
  s / start  → Görevi başlat (TASK1)
  a / abort  → Görevi acil durdur (RTL)
  q / quit   → node'u kapat
  d / status → Anlık durum göster

YAYINLANAN:
-----------
  /swarm/task_trigger   → intent_coordinator görevi alır
  /swarm/gcs_heartbeat  → (std_msgs/String) GCS'nin hayatta olduğunu bildirir

İZLENEN:
---------
  /swarm/intent            → Aktif görev bilgisi (QR no, formasyon, hedef)
  /drone{i}/local_state    → Her drone'un state machine durumu
  /qr/result               → Okunan QR'lar
  /safety/event            → Kritik güvenlik olayları

ŞARTNAME UYUMU:
  §5.1  GCS görev başlatma   → TaskTrigger ile tek komut ✅
  §5.3  Dağıtık mimari       → GCS sadece başlatır; drone'lar otonom ✅
  §5.5  Failsafe gözlemleme  → SafetyEvent dashboard'da görünür ✅
  §18   Görev tamamlama      → COMPLETE → operatöre bildirim ✅
"""

import os
import sys
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String, Header
from swarm_msgs.msg import LocalState, SwarmIntent, TaskTrigger, SafetyEvent, QRResult

# ══════════════════════════════════════════════════════════════════════════════
# DURUM SABİTLERİ
# ══════════════════════════════════════════════════════════════════════════════

class GCSState:
    STANDBY      = 'STANDBY'      # Görev başlamadı
    TASK1_SENT   = 'TASK1_SENT'   # Başlatma komutu gönderildi
    TASK1_ACTIVE = 'TASK1_ACTIVE' # Görev aktif
    ABORT_SENT   = 'ABORT_SENT'   # Durdurma gönderildi
    COMPLETE     = 'COMPLETE'     # Görev tamamlandı


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


# Drone state → terminal renk kodu
_STATE_COLOR: dict[str, str] = {
    DroneState.STANDBY:        '\033[90m',   # gri
    DroneState.IDLE:           '\033[33m',   # sarı
    DroneState.FLYING:         '\033[92m',   # yeşil
    DroneState.DETACH:         '\033[96m',   # cyan
    DroneState.LAND_ZONE:      '\033[94m',   # mavi
    DroneState.DISARM_WAIT:    '\033[35m',   # mor
    DroneState.REARM:          '\033[93m',   # sarı-açık
    DroneState.REJOIN:         '\033[36m',   # cyan-koyu
    DroneState.RETURN_HOME:    '\033[34m',   # mavi-koyu
    DroneState.LANDING:        '\033[95m',   # pembe
    DroneState.SAFETY_HOLD:    '\033[91m',   # kırmızı
    DroneState.PILOT_OVERRIDE: '\033[31m',   # kırmızı-koyu
}
_RESET = '\033[0m'
_BOLD  = '\033[1m'
_RED   = '\033[91m'
_GREEN = '\033[92m'
_YELLOW = '\033[93m'
_CYAN  = '\033[96m'

SWARM_SIZE: int = int(os.environ.get('SWARM_SIZE', '3'))

# Göreve başlatma sonrası FLYING bekleme timeout
FLYING_TIMEOUT_S: float = 60.0

# Tüm dronlar LANDING/DISARM_WAIT ise görev bitti sayılır
COMPLETE_STATES = frozenset({DroneState.LANDING, DroneState.DISARM_WAIT})

# Dashboard yenileme aralığı
DASHBOARD_HZ: float = 1.0


# ══════════════════════════════════════════════════════════════════════════════
# DASHBOARD YARDIMCISI
# ══════════════════════════════════════════════════════════════════════════════

def _state_str(state: str) -> str:
    """Renkli durum etiketi."""
    color = _STATE_COLOR.get(state, '')
    return f'{color}{state:<16}{_RESET}'


def _gcs_state_str(state: str) -> str:
    colors = {
        GCSState.STANDBY:      _YELLOW,
        GCSState.TASK1_SENT:   _CYAN,
        GCSState.TASK1_ACTIVE: _GREEN,
        GCSState.ABORT_SENT:   _RED,
        GCSState.COMPLETE:     '\033[95m',
    }
    return f'{_BOLD}{colors.get(state,"")}{state}{_RESET}'


# ══════════════════════════════════════════════════════════════════════════════
# ANA NODE
# ══════════════════════════════════════════════════════════════════════════════

class MissionFSMNode(Node):
    """
    GCS tarafı görev yöneticisi.

    Operatörün tek komutla görevi başlatmasını/durdurmasını sağlar.
    Tüm İHA'ların durumunu izleyip terminalde gösterir.
    """

    def __init__(self):
        super().__init__('mission_fsm')

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter('team_id',    'team1')
        self.declare_parameter('swarm_size', SWARM_SIZE)

        self._team_id:    str = self.get_parameter('team_id').value
        self._swarm_size: int = self.get_parameter('swarm_size').value

        # ── Dahili durum ──────────────────────────────────────────────────────
        self._gcs_state: str = GCSState.STANDBY

        # drone_id (1-indexed) → state string
        self._drone_states: dict[int, str] = {
            i: DroneState.STANDBY for i in range(1, self._swarm_size + 1)
        }
        # drone_id → last update time
        self._drone_state_times: dict[int, float] = {
            i: 0.0 for i in range(1, self._swarm_size + 1)
        }

        # Son SwarmIntent özeti
        self._last_intent_task_id: str    = '—'
        self._last_intent_formation: str  = '—'
        self._last_intent_qr_seq: int     = 0
        self._last_intent_target: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_intent_time: float     = 0.0

        # Okunan QR'lar (son 10)
        self._qr_history: deque[str] = deque(maxlen=10)

        # Güvenlik olayları (son 5)
        self._safety_events: deque[str] = deque(maxlen=5)

        # Görev başlatma zamanı
        self._task_start_time: float = 0.0
        self._task_elapsed_s:  float = 0.0

        # ── QoS profilleri ────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Yayıncılar ────────────────────────────────────────────────────────
        self._trigger_pub = self.create_publisher(
            TaskTrigger, '/swarm/task_trigger', reliable_qos
        )
        self._heartbeat_pub = self.create_publisher(
            String, '/swarm/gcs_heartbeat', 10
        )

        # ── Abonelikler ───────────────────────────────────────────────────────
        # Tüm drone'ların durumları
        for i in range(1, self._swarm_size + 1):
            self.create_subscription(
                LocalState,
                f'/drone{i}/local_state',
                lambda msg, uid=i: self._on_drone_state(msg, uid),
                reliable_qos,
            )

        # Sürü niyeti (görev ilerlemesi için)
        self.create_subscription(
            SwarmIntent, '/swarm/intent', self._on_intent, 10
        )

        # QR sonuçları
        self.create_subscription(
            QRResult, '/qr/result', self._on_qr_result, 10
        )

        # Güvenlik olayları
        self.create_subscription(
            SafetyEvent, '/safety/event', self._on_safety_event, reliable_qos
        )

        # ── Timer'lar ─────────────────────────────────────────────────────────
        self.create_timer(1.0 / DASHBOARD_HZ, self._dashboard_tick)
        self.create_timer(2.0, self._heartbeat_tick)

        # ── Operatör CLI thread'i ─────────────────────────────────────────────
        self._cli_thread = threading.Thread(
            target=self._cli_loop, daemon=True
        )
        self._cli_thread.start()

        self.get_logger().info(
            f'{_BOLD}mission_fsm başladı{_RESET} — '
            f'team={self._team_id}, swarm_size={self._swarm_size}'
        )
        self._print_help()

    # ══════════════════════════════════════════════════════════════════════════
    # CALLBACK'LER
    # ══════════════════════════════════════════════════════════════════════════

    def _on_drone_state(self, msg: LocalState, uid: int) -> None:
        """Drone durum güncellemesi."""
        self._drone_states[uid] = msg.state
        self._drone_state_times[uid] = time.time()
        self._check_mission_complete()

    def _on_intent(self, msg: SwarmIntent) -> None:
        """Sürü niyeti — görev ilerlemesini takip et."""
        self._last_intent_task_id   = msg.task_id
        self._last_intent_formation = msg.formation_type
        self._last_intent_qr_seq    = int(msg.qr_seq)
        p = msg.target_pos
        self._last_intent_target    = (p.x, p.y, p.z)
        self._last_intent_time      = time.time()

        # TASK1_SENT → TASK1_ACTIVE geçişi
        if (self._gcs_state == GCSState.TASK1_SENT
                and msg.task_id in ('QR_NAVIGATE', 'MANEUVER', 'DETACH', 'REJOIN')):
            self._gcs_state = GCSState.TASK1_ACTIVE
            self.get_logger().info(
                f'{_GREEN}✅ Görev onaylandı: TASK1_ACTIVE{_RESET}'
            )

    def _on_qr_result(self, msg: QRResult) -> None:
        """QR kodu okundu."""
        ts = time.strftime('%H:%M:%S')
        entry = (
            f'[{ts}] QR#{msg.qr_id} '
            f'{msg.formation_type} irtifa={msg.altitude:.0f}m'
            + (f' DETACH=drone{msg.detach_drone_id}' if msg.detach_active else '')
        )
        self._qr_history.append(entry)
        self.get_logger().info(f'📷 {entry}')

    def _on_safety_event(self, msg: SafetyEvent) -> None:
        """Kritik güvenlik olayı."""
        ts = time.strftime('%H:%M:%S')
        entry = (
            f'{_RED}[{ts}] ⚠  drone{msg.drone_id} '
            f'{msg.event_type} sev={msg.severity:.2f}{_RESET}'
        )
        self._safety_events.append(entry)
        self.get_logger().warn(
            f'SAFETY EVENT: drone{msg.drone_id} → {msg.event_type} '
            f'"{msg.description}" (severity={msg.severity:.2f})'
        )

    # ══════════════════════════════════════════════════════════════════════════
    # GÖREV KONTROL
    # ══════════════════════════════════════════════════════════════════════════

    def _send_task_trigger(self, start: bool) -> None:
        """TaskTrigger yayınla."""
        msg = TaskTrigger()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gcs'
        msg.task_type = 'TASK1'
        msg.start     = start
        msg.team_id   = self._team_id
        self._trigger_pub.publish(msg)

    def start_mission(self) -> None:
        """Görevi başlat."""
        if self._gcs_state not in (GCSState.STANDBY, GCSState.COMPLETE):
            print(f'  {_YELLOW}⚠  Görev zaten aktif ({self._gcs_state}){_RESET}')
            return

        # Kalkış öncesi kontrol
        flying_or_idle = sum(
            1 for s in self._drone_states.values()
            if s in (DroneState.IDLE, DroneState.FLYING, DroneState.STANDBY)
        )
        if flying_or_idle == 0:
            print(f'  {_RED}❌ Drone durumları alınamadı. Dronlar bağlı mı?{_RESET}')
            return

        self._send_task_trigger(start=True)
        self._gcs_state = GCSState.TASK1_SENT
        self._task_start_time = time.time()
        print(
            f'\n  {_GREEN}🚀 GÖREV BAŞLATILDI{_RESET} — '
            f'team={self._team_id}\n'
            f'  intent_coordinator TASK1_ACTIVE onayı bekleniyor...\n'
        )

    def abort_mission(self) -> None:
        """Görevi acil durdur."""
        if self._gcs_state == GCSState.STANDBY:
            print(f'  {_YELLOW}⚠  Görev zaten başlamadı{_RESET}')
            return

        self._send_task_trigger(start=False)
        self._gcs_state = GCSState.ABORT_SENT
        print(
            f'\n  {_RED}⛔ GÖREV DURDURULDU{_RESET} — '
            f'Dronlar RTL moduna geçiyor...\n'
        )

    def _check_mission_complete(self) -> None:
        """Tüm dronlar LANDING veya DISARM_WAIT → COMPLETE."""
        if self._gcs_state not in (GCSState.TASK1_ACTIVE, GCSState.ABORT_SENT):
            return

        if not self._drone_states:
            return

        all_done = all(
            s in COMPLETE_STATES
            for s in self._drone_states.values()
        )
        if all_done:
            self._gcs_state = GCSState.COMPLETE
            elapsed = time.time() - self._task_start_time
            self.get_logger().info(
                f'{_GREEN}🏁 GÖREV TAMAMLANDI — '
                f'Süre: {elapsed:.0f}s{_RESET}'
            )

    # ══════════════════════════════════════════════════════════════════════════
    # DASHBOARD
    # ══════════════════════════════════════════════════════════════════════════

    def _dashboard_tick(self) -> None:
        """1 Hz — terminal dashboardunu güncelle."""
        if self._gcs_state == GCSState.TASK1_ACTIVE and self._task_start_time:
            self._task_elapsed_s = time.time() - self._task_start_time

        self._print_dashboard()

        # FLYING_TIMEOUT kontrolü
        if self._gcs_state == GCSState.TASK1_SENT:
            elapsed = time.time() - self._task_start_time
            if elapsed > FLYING_TIMEOUT_S:
                self.get_logger().warn(
                    f'{_YELLOW}⏰ FLYING_TIMEOUT ({FLYING_TIMEOUT_S}s) — '
                    f'dronlar FLYING\'e geçmedi!{_RESET}'
                )
                self._gcs_state = GCSState.STANDBY

    def _print_dashboard(self) -> None:
        """Terminal dashboardunu bas (ANSI escape ile yerinde güncelle)."""
        now_str = time.strftime('%H:%M:%S')

        lines = []
        lines.append(
            f'\033[2J\033[H'   # Ekranı temizle, başa git
        )
        lines.append(
            f'{_BOLD}╔══════════════════════════════════════════════╗{_RESET}'
        )
        lines.append(
            f'{_BOLD}║   TEKNOFEST Sürü İHA — GCS Mission FSM      ║{_RESET}'
        )
        lines.append(
            f'{_BOLD}╚══════════════════════════════════════════════╝{_RESET}'
        )
        lines.append(f'  Saat    : {now_str}')
        lines.append(f'  Team    : {_BOLD}{self._team_id}{_RESET}')
        lines.append(
            f'  GCS     : {_gcs_state_str(self._gcs_state)}'
            + (f'  ⏱ {self._task_elapsed_s:.0f}s'
               if self._gcs_state == GCSState.TASK1_ACTIVE else '')
        )
        lines.append('')

        # ── Drone durumları ───────────────────────────────────────────────────
        lines.append(f'  {_BOLD}DRONE DURUMU{_RESET}')
        lines.append('  ' + '─' * 40)
        now = time.time()
        for did in sorted(self._drone_states.keys()):
            state = self._drone_states[did]
            age = now - self._drone_state_times[did]
            age_str = f'{age:.0f}s önce' if age < 120 else '—'
            stale_warn = f' {_RED}⚠ STALE{_RESET}' if age > 2.0 else ''
            lines.append(
                f'  drone{did}  {_state_str(state)}'
                f'  {_YELLOW}{age_str}{_RESET}{stale_warn}'
            )
        lines.append('')

        # ── Aktif görev ───────────────────────────────────────────────────────
        if self._gcs_state in (GCSState.TASK1_SENT, GCSState.TASK1_ACTIVE):
            lines.append(f'  {_BOLD}AKTİF GÖREV{_RESET}')
            lines.append('  ' + '─' * 40)
            tx, ty, tz = self._last_intent_target
            lines.append(f'  Görev    : {self._last_intent_task_id}')
            lines.append(f'  Formasyon: {self._last_intent_formation}')
            lines.append(f'  QR Sıra  : {self._last_intent_qr_seq}')
            lines.append(
                f'  Hedef    : ({tx:.1f}, {ty:.1f}, {tz:.1f}) m'
            )
            intent_age = now - self._last_intent_time
            if intent_age > 3.0:
                lines.append(
                    f'  {_YELLOW}⚠ Intent {intent_age:.0f}s önce — '
                    f'intent_coordinator bağlantısı?{_RESET}'
                )
            lines.append('')

        # ── Son QR okumaları ─────────────────────────────────────────────────
        if self._qr_history:
            lines.append(f'  {_BOLD}SON QR OKUMALAR{_RESET}')
            lines.append('  ' + '─' * 40)
            for entry in list(self._qr_history)[-3:]:
                lines.append(f'  {entry}')
            lines.append('')

        # ── Güvenlik olayları ─────────────────────────────────────────────────
        if self._safety_events:
            lines.append(f'  {_BOLD}GÜVENLİK OLAYLARI{_RESET}')
            lines.append('  ' + '─' * 40)
            for entry in list(self._safety_events)[-3:]:
                lines.append(f'  {entry}')
            lines.append('')

        # ── COMPLETE mesajı ───────────────────────────────────────────────────
        if self._gcs_state == GCSState.COMPLETE:
            elapsed = time.time() - self._task_start_time
            lines.append(
                f'  {_GREEN}{_BOLD}🏁 GÖREV TAMAMLANDI! '
                f'Toplam süre: {elapsed:.0f}s{_RESET}'
            )
            lines.append('')

        # ── Komutlar ──────────────────────────────────────────────────────────
        lines.append('  ' + '─' * 40)
        lines.append(
            f'  Komutlar: '
            f'{_BOLD}[s]{_RESET}tart  '
            f'{_BOLD}[a]{_RESET}bort  '
            f'{_BOLD}[q]{_RESET}uit'
        )
        lines.append('  > ')

        print('\n'.join(lines), end='', flush=True)

    def _print_help(self) -> None:
        print(
            f'\n{_BOLD}╔══════════════════════════════════════╗{_RESET}'
            f'\n{_BOLD}║  mission_fsm — Komut listesi         ║{_RESET}'
            f'\n{_BOLD}╚══════════════════════════════════════╝{_RESET}'
            f'\n  {_BOLD}s{_RESET} / start  → TASK1 başlat'
            f'\n  {_BOLD}a{_RESET} / abort  → Görevi acil durdur (RTL)'
            f'\n  {_BOLD}d{_RESET} / status → Dashboard'
            f'\n  {_BOLD}q{_RESET} / quit   → Çıkış'
            f'\n'
        )

    # ══════════════════════════════════════════════════════════════════════════
    # HEARTBEAT
    # ══════════════════════════════════════════════════════════════════════════

    def _heartbeat_tick(self) -> None:
        """2 Hz — GCS'nin aktif olduğunu yayınla."""
        msg = String()
        msg.data = f'{self._gcs_state}:{int(time.time())}'
        self._heartbeat_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════════════
    # CLI LOOP (ayrı thread)
    # ══════════════════════════════════════════════════════════════════════════

    def _cli_loop(self) -> None:
        """
        Operatör komutlarını dinle (ayrı thread).
        rclpy.spin() ana thread'i meşgul ettiği için CLI için thread lazım.
        """
        try:
            while rclpy.ok():
                try:
                    cmd = input('').strip().lower()
                except EOFError:
                    break

                if cmd in ('s', 'start'):
                    self.start_mission()
                elif cmd in ('a', 'abort'):
                    self.abort_mission()
                elif cmd in ('d', 'status'):
                    self._print_dashboard()
                elif cmd in ('q', 'quit', 'exit'):
                    self.get_logger().info('mission_fsm kapatılıyor...')
                    rclpy.shutdown()
                    break
                elif cmd == '':
                    pass  # Enter'a basmak dashboard yeniler
                else:
                    print(
                        f'  {_YELLOW}Bilinmeyen komut: "{cmd}". '
                        f'[s]tart / [a]bort / [q]uit{_RESET}'
                    )
        except Exception as e:
            self.get_logger().error(f'CLI hatası: {e}')


# ══════════════════════════════════════════════════════════════════════════════
# ENTRYPOINT
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = MissionFSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\n  {_YELLOW}Ctrl+C — mission_fsm kapatılıyor{_RESET}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ══════════════════════════════════════════════════════════════════════════════
# ÇALIŞTIRMA
# ══════════════════════════════════════════════════════════════════════════════
#
# Temel kullanım:
#   ros2 run my_swarm_pkg mission_fsm \
#     --ros-args -p team_id:=team1 -p swarm_size:=3
#
# Görev başlatma (CLI olmadan, tek komutla):
#   ros2 topic pub --once /swarm/task_trigger swarm_msgs/msg/TaskTrigger \
#     "{task_type: 'TASK1', start: true, team_id: 'team1'}"
#
# Durum izleme (mission_fsm olmadan):
#   watch -n 1 "ros2 topic echo /swarm/intent --once 2>/dev/null"
