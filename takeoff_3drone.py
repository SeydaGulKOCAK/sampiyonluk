#!/usr/bin/env python3
"""
╔═══════════════════════════════════════════════════════════╗
║   3 DRONE SENKRON KALKIŞ — MAVROS 2.x UYUMLU             ║
║   İNŞAALLAH UÇARLAR! 🙏🚁🚁🚁                           ║
╚═══════════════════════════════════════════════════════════╝

KULLANIM:
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_LOCALHOST_ONLY=1
  python3 ~/sampiyonluk/takeoff_3drone.py
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandLong, CommandTOL
from geometry_msgs.msg import PoseStamped
import time
import sys
import os

# Ortam değişkenlerini ayarla
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')


class TakeoffNode(Node):
    """3 drone'u GUIDED modda kaldırır."""

    # MAVROS 2.x topic formatı: /droneX/<topic>  (mavros/ prefix yok!)
    DRONES = ['drone1', 'drone2', 'drone3']

    def __init__(self):
        super().__init__('takeoff_3drone')
        self.get_logger().info('🚁 3 Drone Takeoff başlatılıyor...')

        self.states = {}
        self.positions = {}
        self.mode_clients = {}
        self.arm_clients = {}
        self.takeoff_clients = {}
        self.cmd_clients = {}

        for ns in self.DRONES:
            self.states[ns] = None
            self.positions[ns] = {'x': 0.0, 'y': 0.0, 'z': 0.0}

            # MAVROS 2.x: /droneX/state (not /droneX/mavros/state)
            self.create_subscription(
                State, f'/{ns}/state',
                lambda msg, n=ns: self._state_cb(n, msg), 10
            )

            self.create_subscription(
                PoseStamped, f'/{ns}/local_position/pose',
                lambda msg, n=ns: self._pose_cb(n, msg), 10
            )

            # Services: MAVROS 2.x formatı
            self.mode_clients[ns] = self.create_client(
                SetMode, f'/{ns}/set_mode'
            )
            self.arm_clients[ns] = self.create_client(
                CommandBool, f'/{ns}/mavros/arming'
            )
            self.takeoff_clients[ns] = self.create_client(
                CommandTOL, f'/{ns}/mavros/takeoff'
            )
            self.cmd_clients[ns] = self.create_client(
                CommandLong, f'/{ns}/mavros/command'
            )

    def _state_cb(self, ns, msg):
        self.states[ns] = msg

    def _pose_cb(self, ns, msg):
        self.positions[ns] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
        }

    def wait_connections(self, timeout=60):
        """MAVROS bağlantısını bekle."""
        print("\n━━━ ADIM 1: BAĞLANTI BEKLENİYOR ━━━")
        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            connected = [ns for ns in self.DRONES
                         if self.states[ns] and self.states[ns].connected]
            elapsed = int(time.time() - start)
            icons = ' '.join([
                f"{'✅' if ns in connected else '⏳'}{ns}"
                for ns in self.DRONES
            ])
            print(f"\r  [{elapsed:2d}s] {icons}   ", end='', flush=True)
            if len(connected) == 3:
                print(f"\n  ✅ 3/3 DRONE BAĞLI!")
                return True
        print(f"\n  ❌ Timeout!")
        return False

    def set_guided(self):
        """3 drone'u AYNI ANDA GUIDED moda geçir."""
        print("\n━━━ ADIM 2: GUIDED MOD (EŞZAMANLI) ━━━")

        # Önce servislerin hazır olmasını bekle
        for ns in self.DRONES:
            self.mode_clients[ns].wait_for_service(timeout_sec=10.0)

        # 3'üne AYNI ANDA gönder
        futures = {}
        for ns in self.DRONES:
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            futures[ns] = self.mode_clients[ns].call_async(req)
        print("  📡 3 drone'a AYNI ANDA GUIDED komutu gönderildi!")

        # Hepsinin cevabını bekle
        for ns, future in futures.items():
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() and future.result().mode_sent:
                print(f"  ✅ {ns} → GUIDED")
            else:
                print(f"  ⚠️  {ns} GUIDED belirsiz")

        time.sleep(2)
        for ns in self.DRONES:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.states[ns]:
                print(f"  📋 {ns} mode: {self.states[ns].mode}")

    def arm_all(self):
        """3 drone'u AYNI ANDA ARM et."""
        print("\n━━━ ADIM 3: ARM 🔥 (EŞZAMANLI) ━━━")

        # Önce servislerin hazır olmasını bekle
        for ns in self.DRONES:
            self.arm_clients[ns].wait_for_service(timeout_sec=10.0)

        # 3'üne AYNI ANDA ARM gönder
        futures = {}
        for ns in self.DRONES:
            req = CommandBool.Request()
            req.value = True
            futures[ns] = self.arm_clients[ns].call_async(req)
        print("  📡 3 drone'a AYNI ANDA ARM komutu gönderildi!")

        # Hepsinin cevabını bekle
        for ns, future in futures.items():
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() and future.result().success:
                print(f"  ✅ {ns} → ARMED 🔥")
            else:
                print(f"  ⚠️  {ns} ARM belirsiz, tekrar deneniyor...")

        # Başarısız olanları tekrar dene (yine eşzamanlı)
        time.sleep(2)
        retry = {}
        for ns in self.DRONES:
            rclpy.spin_once(self, timeout_sec=0.3)
            if not (self.states[ns] and self.states[ns].armed):
                req = CommandBool.Request()
                req.value = True
                retry[ns] = self.arm_clients[ns].call_async(req)
        if retry:
            print(f"  🔁 {len(retry)} drone tekrar ARM ediliyor...")
            for ns, future in retry.items():
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        time.sleep(2)
        # Final kontrol
        for ns in self.DRONES:
            rclpy.spin_once(self, timeout_sec=0.3)
            armed = self.states[ns].armed if self.states[ns] else False
            icon = "🔥" if armed else "⚠️"
            print(f"  {icon} {ns} armed={armed}")

    def takeoff(self, altitude=5.0):
        """3 drone'u AYNI ANDA kaldır."""
        print(f"\n━━━ ADIM 4: TAKEOFF → {altitude}m 🚀 (EŞZAMANLI) ━━━")

        # Önce servislerin hazır olmasını bekle
        for ns in self.DRONES:
            self.takeoff_clients[ns].wait_for_service(timeout_sec=10.0)

        # 3'üne AYNI ANDA TAKEOFF gönder
        futures = {}
        for ns in self.DRONES:
            req = CommandTOL.Request()
            req.altitude = float(altitude)
            req.latitude = 0.0
            req.longitude = 0.0
            req.min_pitch = 0.0
            req.yaw = 0.0
            futures[ns] = self.takeoff_clients[ns].call_async(req)
        print("  📡 3 drone'a AYNI ANDA TAKEOFF komutu gönderildi!")

        # Hepsinin cevabını bekle
        for ns, future in futures.items():
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() and future.result().success:
                print(f"  🚀 {ns} TAKEOFF KABUL EDİLDİ!")
            else:
                print(f"  ⚠️  {ns} takeoff belirsiz")

        # Yükselmeyi takip et
        print()
        start = time.time()
        all_up = False
        while rclpy.ok() and (time.time() - start) < 30:
            rclpy.spin_once(self, timeout_sec=0.3)
            parts = []
            alts = []
            for ns in self.DRONES:
                z = self.positions[ns]['z']
                alts.append(z)
                icon = "✅" if z > altitude * 0.7 else "⬆️"
                parts.append(f"{icon}{ns}={z:.1f}m")
            print(f"\r  {' | '.join(parts)}    ", end='', flush=True)
            if all(z > altitude * 0.7 for z in alts):
                all_up = True
                break

        print()
        if all_up:
            print(f"\n  🎉🎉🎉 3 DRONE HAVADA! ALHAMDÜLİLLAH! 🎉🎉🎉")
        else:
            print(f"\n  ⚠️  Bazı drone'lar henüz yeterli yüksekliğe ulaşamadı")
            # Tekrar dene
            for ns in self.DRONES:
                if self.positions[ns]['z'] < altitude * 0.5:
                    req = CommandTOL.Request()
                    req.altitude = float(altitude)
                    req.latitude = 0.0
                    req.longitude = 0.0
                    req.min_pitch = 0.0
                    req.yaw = 0.0
                    self.takeoff_clients[ns].call_async(req)
                    print(f"  🔁 {ns} tekrar TAKEOFF")

        return all_up

    def hover_monitor(self):
        """Havada kal, pozisyonları göster."""
        print("\n━━━ HOVER MOD (Ctrl+C ile çık) ━━━\n")
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.5)
                parts = []
                for ns in self.DRONES:
                    p = self.positions[ns]
                    parts.append(f"{ns}=({p['x']:.1f},{p['y']:.1f},{p['z']:.1f})")
                print(f"\r  📍 {' | '.join(parts)}  ", end='', flush=True)
        except KeyboardInterrupt:
            print("\n\n  ⛔ Durduruldu.")


def main():
    rclpy.init()
    node = TakeoffNode()

    print()
    print("╔═══════════════════════════════════════════════════════╗")
    print("║  🚁🚁🚁 3 DRONE SENKRON KALKIŞ — BİSMİLLAH! 🙏    ║")
    print("╚═══════════════════════════════════════════════════════╝")

    try:
        # 1) Bağlantı bekle
        if not node.wait_connections(timeout=30):
            print("\n❌ Bağlantı kurulamadı!")
            return

        # 2) GUIDED mod
        node.set_guided()
        time.sleep(2)

        # 3) ARM
        node.arm_all()
        time.sleep(2)

        # 4) TAKEOFF
        alt = 5.0
        try:
            val = input(f"\n  Kalkış yüksekliği [{alt}m] (ENTER=varsayılan): ").strip()
            if val:
                alt = float(val)
        except (ValueError, EOFError):
            pass

        node.takeoff(altitude=alt)

        # 5) Hover
        node.hover_monitor()

    except KeyboardInterrupt:
        print("\n\n⛔ Durduruldu.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
