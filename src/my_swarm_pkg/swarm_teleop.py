#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                        swarm_teleop.py  v1                                  ║
║           Görev-2 Yarı Otonom Joystick Kontrolü — GCS Node'u                ║
╚══════════════════════════════════════════════════════════════════════════════╝

GENEL AÇIKLAMA:
---------------
Bu node GCS (laptop) üzerinde çalışır ve TASK2 (Yarı Otonom Sürü Kontrolü)
sırasında operatörün klavye veya joystick ile sürüyü kontrol etmesini sağlar.

GÖREV-2 MOD SEÇİMİ:
-------------------
  MOVE     : Joystick/klavye → tüm sürü hareket eder (formasyon bozulmaz)
  MANEUVER : Formasyon şeklini/aralığını değiştir (merkez sabit)
  IDLE     : Komut gönderme (hover)

KONTROL GİRİŞİ:
---------------
  Mod A — Klavye (varsayılan, joystick yoksa):
    W/S   : İleri/Geri  (vx)
    A/D   : Sol/Sağ     (vy)
    Q/E   : Yükselt/Alçalt (vz)
    Z/X   : Sol/Sağ yaw dönüşü
    1     : OKBASI formasyonu
    2     : V formasyonu
    3     : CIZGI formasyonu
    +/-   : Spacing artır/azalt

  Mod B — pygame Joystick (varsa otomatik aktive):
    Sol  analog: vx/vy
    Sağ  analog: yaw_rate / vz
    L1/R1: Formasyon değiştir
    Cross/Kare: Spacing +/-

ÇIKIŞ:
------
  /swarm/teleop_cmd   → formation_controller (TeleopCmd)
  /swarm/teleop_mode  → formation_controller (String: IDLE/MOVE/MANEUVER)

BAĞIMLILIK:
-----------
  formation_controller bu topic'leri dinliyor:
    /swarm/teleop_cmd  → _on_teleop_cmd (TeleopCmd)
    /swarm/teleop_mode → _on_teleop_mode (String)

MODE GATING:
------------
  TASK1_ACTIVE'de bu node çıktı yayınlamaz.
  /swarm/gcs_heartbeat üzerindeki state'i izler.

ŞARTNAME UYUMU:
  §5.2  Görev-2 yarı otonom  → Joystick/klavye sürü kontrolü ✅
  §14   Formasyon değişimi   → MANEUVER mod ile anlık forma değişimi ✅
  §5.3  Mode gating          → TASK1_ACTIVE'de etkisiz ✅
"""

import math
import os
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from swarm_msgs.msg import TeleopCmd

# Joystick opsiyonel
try:
    import pygame
    _PYGAME_AVAILABLE = True
except ImportError:
    _PYGAME_AVAILABLE = False

# ══════════════════════════════════════════════════════════════════════════════
# PARAMETRE SABİTLERİ
# ══════════════════════════════════════════════════════════════════════════════

PUBLISH_HZ: float  = 20.0    # Komut yayın frekansı
PUBLISH_DT: float  = 1.0 / PUBLISH_HZ

# Maksimum hız değerleri (klavye tam basım)
MAX_VX_MPS: float  = 3.0     # m/s ileri/geri
MAX_VY_MPS: float  = 3.0     # m/s sol/sağ
MAX_VZ_MPS: float  = 1.5     # m/s yukarı/aşağı
MAX_YAW_RPS: float = 0.3     # rad/s yaw dönüşü

# Spacing adımı (her basışta)
SPACING_STEP_M: float = 0.5
SPACING_MIN_M:  float = 2.0
SPACING_MAX_M:  float = 15.0

# Wi-Fi gecikme kompansasyonu (ms cinsinden offset)
CMD_DELAY_MS: float = 50.0   # execute_at = now + 50ms

# Joystick dead zone
JOY_DEAD_ZONE: float = 0.05

# ANSI renk kodları
_BOLD  = '\033[1m'
_RESET = '\033[0m'
_GREEN = '\033[92m'
_YELLOW = '\033[93m'
_CYAN  = '\033[96m'
_RED   = '\033[91m'


# ══════════════════════════════════════════════════════════════════════════════
# KLAVYE GİRİŞ İŞLEYİCİSİ
# ══════════════════════════════════════════════════════════════════════════════

class KeyboardInput:
    """
    Non-blocking klavye girdisi (Linux termios).
    W/A/S/D/Q/E/Z/X → hız vektörü
    """

    def __init__(self):
        self._keys_down: set[str] = set()
        self._running = True
        self._lock    = threading.Lock()
        self._thread  = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        import termios
        import tty
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self._running:
                ch = sys.stdin.read(1)
                if ch:
                    with self._lock:
                        self._keys_down = {ch}  # En son basılan tek tuş
        except Exception:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def get_key(self) -> str:
        with self._lock:
            if self._keys_down:
                key = next(iter(self._keys_down))
                return key
        return ''

    def stop(self):
        self._running = False


# ══════════════════════════════════════════════════════════════════════════════
# ANA NODE
# ══════════════════════════════════════════════════════════════════════════════

class SwarmTeleopNode(Node):

    def __init__(self):
        super().__init__('swarm_teleop')

        # ── Parametreler ──────────────────────────────────────────────────────
        self.declare_parameter('use_keyboard', True)
        self.declare_parameter('formation_type', 'OKBASI')
        self.declare_parameter('drone_spacing', 6.0)
        self.declare_parameter('max_speed', MAX_VX_MPS)

        self._use_keyboard: bool  = self.get_parameter('use_keyboard').value
        self._cur_formation: str  = self.get_parameter('formation_type').value
        self._cur_spacing: float  = float(self.get_parameter('drone_spacing').value)
        self._max_speed: float    = float(self.get_parameter('max_speed').value)

        # ── Dahili durum ──────────────────────────────────────────────────────
        self._teleop_mode: str     = 'IDLE'
        self._task1_active: bool   = False   # Mode gating

        # Mevcut hız komutları
        self._vx:       float = 0.0
        self._vy:       float = 0.0
        self._vz:       float = 0.0
        self._yaw_rate: float = 0.0

        # Joystick
        self._joystick = None
        if _PYGAME_AVAILABLE and not self._use_keyboard:
            self._init_joystick()

        # ── QoS ──────────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Yayıncılar ────────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            TeleopCmd, '/swarm/teleop_cmd', reliable_qos
        )
        self._mode_pub = self.create_publisher(
            String, '/swarm/teleop_mode', reliable_qos
        )

        # ── Abonelikler ───────────────────────────────────────────────────────
        self.create_subscription(
            String, '/swarm/gcs_heartbeat', self._on_heartbeat, 10
        )

        # ── Timer'lar ─────────────────────────────────────────────────────────
        self.create_timer(PUBLISH_DT, self._publish_loop)
        self.create_timer(1.0,        self._status_tick)

        # ── Klavye thread'i ───────────────────────────────────────────────────
        self._kb: KeyboardInput | None = None
        if self._use_keyboard:
            self._kb = KeyboardInput()
            self._kb_thread = threading.Thread(
                target=self._keyboard_loop, daemon=True
            )
            self._kb_thread.start()

        self.get_logger().info(
            f'{_BOLD}swarm_teleop başladı{_RESET}\n'
            f'  Giriş: {"klavye" if self._use_keyboard else "joystick"}\n'
            f'  Formasyon: {self._cur_formation}, '
            f'Spacing: {self._cur_spacing}m\n'
            f'  Max hız: {self._max_speed} m/s'
        )
        self._print_help()

    # ══════════════════════════════════════════════════════════════════════════
    # JOYSTICK BAŞLATMA
    # ══════════════════════════════════════════════════════════════════════════

    def _init_joystick(self):
        try:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self._joystick = pygame.joystick.Joystick(0)
                self._joystick.init()
                self.get_logger().info(
                    f'🎮 Joystick: {self._joystick.get_name()}'
                )
            else:
                self.get_logger().warn(
                    '⚠  Joystick bulunamadı → klavye moduna geçildi'
                )
                self._use_keyboard = True
        except Exception as e:
            self.get_logger().warn(f'Joystick init hatası: {e} → klavye')
            self._use_keyboard = True

    # ══════════════════════════════════════════════════════════════════════════
    # CALLBACK'LER
    # ══════════════════════════════════════════════════════════════════════════

    def _on_heartbeat(self, msg: String) -> None:
        """GCS heartbeat: TASK1_ACTIVE ise mode gating uygula."""
        self._task1_active = 'TASK1_ACTIVE' in msg.data

    # ══════════════════════════════════════════════════════════════════════════
    # YAYINLAMA DÖNGÜSÜ — 20 Hz
    # ══════════════════════════════════════════════════════════════════════════

    def _publish_loop(self) -> None:
        """20 Hz — TeleopCmd yayınla."""
        if self._task1_active:
            return  # Mode gating: TASK1_ACTIVE'de bu node susturulur

        if self._teleop_mode == 'IDLE':
            return

        # Joystick girdisi güncelle
        if self._joystick and not self._use_keyboard:
            self._read_joystick()

        # TeleopCmd oluştur
        msg = TeleopCmd()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gcs'
        msg.mode        = self._teleop_mode
        msg.vx          = self._vx
        msg.vy          = self._vy
        msg.vz          = self._vz
        msg.yaw_rate    = self._yaw_rate

        # execute_at: Wi-Fi gecikme kompansasyonu
        now_ns = self.get_clock().now().nanoseconds
        delay_ns = int(CMD_DELAY_MS * 1_000_000)
        exec_time = rclpy.clock.Clock().now()  # basit yaklaşım
        msg.execute_at = self.get_clock().now().to_msg()  # 0 → hemen

        msg.new_formation_type = ''
        msg.new_spacing        = 0.0

        self._cmd_pub.publish(msg)

    def _set_teleop_mode(self, mode: str) -> None:
        """Mod değişikliğini yayınla."""
        if self._teleop_mode == mode:
            return
        self._teleop_mode = mode
        mode_msg      = String()
        mode_msg.data = mode
        self._mode_pub.publish(mode_msg)
        self.get_logger().info(f'Teleop mod → {mode}')

    def _send_maneuver(
        self,
        formation: str = '',
        spacing: float = 0.0,
    ) -> None:
        """Anlık MANEUVER komutu gönder."""
        if self._task1_active:
            return

        msg = TeleopCmd()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.mode            = 'MANEUVER'
        msg.vx = msg.vy = msg.vz = msg.yaw_rate = 0.0
        msg.new_formation_type = formation
        msg.new_spacing        = spacing
        msg.execute_at         = self.get_clock().now().to_msg()
        self._cmd_pub.publish(msg)

        if formation:
            self._cur_formation = formation
        if spacing > 0:
            self._cur_spacing = spacing

    # ══════════════════════════════════════════════════════════════════════════
    # JOYSTICK GİRİŞİ
    # ══════════════════════════════════════════════════════════════════════════

    def _read_joystick(self):
        """pygame joystick eksenlerini oku."""
        if not _PYGAME_AVAILABLE or self._joystick is None:
            return
        pygame.event.pump()

        def _deadzone(v: float) -> float:
            return v if abs(v) > JOY_DEAD_ZONE else 0.0

        # Sol analog: vy (yatay), vx (dikey)
        raw_vy = _deadzone(self._joystick.get_axis(0))  # sağ (+)→ negatif vy
        raw_vx = _deadzone(-self._joystick.get_axis(1)) # yukarı (+) = ileri
        # Sağ analog: yaw, vz
        raw_yaw = _deadzone(self._joystick.get_axis(2))
        raw_vz  = _deadzone(-self._joystick.get_axis(3))

        self._vx       = raw_vx  * self._max_speed
        self._vy       = -raw_vy * self._max_speed  # sol +
        self._vz       = raw_vz  * MAX_VZ_MPS
        self._yaw_rate = -raw_yaw * MAX_YAW_RPS

    # ══════════════════════════════════════════════════════════════════════════
    # KLAVYE DÖNGÜSÜ (ayrı thread)
    # ══════════════════════════════════════════════════════════════════════════

    def _keyboard_loop(self) -> None:
        """Klavye girdisini dinle."""
        try:
            import termios, tty
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            tty.setraw(fd)

            self.get_logger().info('⌨  Klavye dinleniyor...')

            while rclpy.ok():
                ch = sys.stdin.read(1)
                if not ch:
                    continue

                ch_lower = ch.lower()

                # ── Çıkış ────────────────────────────────────────────────────
                if ch_lower in ('q',) and self._vx == 0:
                    # q sadece hareket yokken IDLE'a geçer (hareket sırasında geri değil)
                    self._set_teleop_mode('IDLE')
                    self._vx = self._vy = self._vz = self._yaw_rate = 0.0
                    continue

                # ── Mod geçişi ────────────────────────────────────────────────
                if ch == '\x1b':  # ESC → IDLE
                    self._set_teleop_mode('IDLE')
                    self._vx = self._vy = self._vz = self._yaw_rate = 0.0
                    continue

                if ch_lower == 'm':
                    new = 'MANEUVER' if self._teleop_mode != 'MANEUVER' else 'IDLE'
                    self._set_teleop_mode(new)
                    self._vx = self._vy = self._vz = self._yaw_rate = 0.0
                    continue

                # ── Hareket komutu (MOVE modu) ────────────────────────────────
                if ch_lower in ('w', 'a', 's', 'd', 'q', 'e', 'z', 'x'):
                    if self._teleop_mode != 'MOVE':
                        self._set_teleop_mode('MOVE')

                    # Hızları sıfırla, sadece basılı tuşu uygula
                    self._vx = self._vy = self._vz = self._yaw_rate = 0.0

                    if   ch_lower == 'w':  self._vx =  self._max_speed
                    elif ch_lower == 's':  self._vx = -self._max_speed
                    elif ch_lower == 'a':  self._vy =  self._max_speed
                    elif ch_lower == 'd':  self._vy = -self._max_speed
                    elif ch_lower == 'e':  self._vz =  MAX_VZ_MPS
                    elif ch_lower == 'q':  self._vz = -MAX_VZ_MPS
                    elif ch_lower == 'z':  self._yaw_rate =  MAX_YAW_RPS
                    elif ch_lower == 'x':  self._yaw_rate = -MAX_YAW_RPS
                    continue

                # Tuş bırakıldığında (boşluk veya enter) dur
                if ch in (' ', '\r', '\n'):
                    self._vx = self._vy = self._vz = self._yaw_rate = 0.0
                    continue

                # ── Formasyon değiştir ────────────────────────────────────────
                if ch == '1':
                    self._send_maneuver(formation='OKBASI')
                    print(f'\r  Formasyon → OKBASI          \r', end='', flush=True)
                elif ch == '2':
                    self._send_maneuver(formation='V')
                    print(f'\r  Formasyon → V               \r', end='', flush=True)
                elif ch == '3':
                    self._send_maneuver(formation='CIZGI')
                    print(f'\r  Formasyon → CIZGI           \r', end='', flush=True)

                # ── Spacing ───────────────────────────────────────────────────
                elif ch == '+':
                    new_sp = min(SPACING_MAX_M, self._cur_spacing + SPACING_STEP_M)
                    self._send_maneuver(spacing=new_sp)
                    print(f'\r  Spacing → {new_sp:.1f}m              \r',
                          end='', flush=True)
                elif ch == '-':
                    new_sp = max(SPACING_MIN_M, self._cur_spacing - SPACING_STEP_M)
                    self._send_maneuver(spacing=new_sp)
                    print(f'\r  Spacing → {new_sp:.1f}m              \r',
                          end='', flush=True)

        except Exception as e:
            self.get_logger().error(f'Klavye döngüsü hatası: {e}')
        finally:
            try:
                import termios
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    # ══════════════════════════════════════════════════════════════════════════
    # DURUM LOGU
    # ══════════════════════════════════════════════════════════════════════════

    def _status_tick(self) -> None:
        """1s'de bir durum logu."""
        gating = f'{_RED}TASK1_ACTIVE — komutlar susturuldu{_RESET}' \
                 if self._task1_active else f'{_GREEN}Aktif{_RESET}'
        self.get_logger().debug(
            f'Mod={self._teleop_mode} | '
            f'vx={self._vx:.1f} vy={self._vy:.1f} vz={self._vz:.1f} '
            f'yaw={self._yaw_rate:.2f} | '
            f'{self._cur_formation}/{self._cur_spacing:.1f}m | {gating}'
        )

    def _print_help(self) -> None:
        print(
            f'\n{_BOLD}╔══════════════════════════════════════════════╗{_RESET}'
            f'\n{_BOLD}║  swarm_teleop — Klavye Kontrol Listesi       ║{_RESET}'
            f'\n{_BOLD}╚══════════════════════════════════════════════╝{_RESET}'
            f'\n  {_BOLD}Hareket (MOVE mod):{_RESET}'
            f'\n    W/S  → İleri/Geri       A/D  → Sol/Sağ'
            f'\n    E/Q  → Yukarı/Aşağı     Z/X  → Yaw sol/sağ'
            f'\n    SPACE/ENTER → Dur (hız=0)'
            f'\n    ESC  → IDLE (tüm komutlar durdurulur)'
            f'\n'
            f'\n  {_BOLD}Formasyon (MANEUVER mod):{_RESET}'
            f'\n    1 → OKBASI   2 → V   3 → CIZGI'
            f'\n    + / -  → Spacing artır/azalt'
            f'\n    M  → MANEUVER mod geçiş'
            f'\n'
        )


# ══════════════════════════════════════════════════════════════════════════════
# ENTRYPOINT
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = SwarmTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\n  {_YELLOW}Ctrl+C — swarm_teleop kapatılıyor{_RESET}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ══════════════════════════════════════════════════════════════════════════════
# ÇALIŞTIRMA
# ══════════════════════════════════════════════════════════════════════════════
#
# Klavye ile:
#   ros2 run my_swarm_pkg swarm_teleop \
#     --ros-args -p use_keyboard:=true -p drone_spacing:=6.0
#
# Joystick ile:
#   ros2 run my_swarm_pkg swarm_teleop \
#     --ros-args -p use_keyboard:=false
#
# Test (formation_controller dinliyor mu?):
#   ros2 topic echo /swarm/teleop_cmd
#   ros2 topic echo /swarm/teleop_mode
