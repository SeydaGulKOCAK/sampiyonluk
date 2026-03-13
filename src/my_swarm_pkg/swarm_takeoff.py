#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from geometry_msgs.msg import PoseStamped


class SwarmTakeoff(Node):

    def __init__(self):
        super().__init__('swarm_takeoff')

        self.drones = ['drone1', 'drone2', 'drone3']

        self.states = {}
        self.altitudes = {}

        self.mode_clients = {}
        self.arm_clients = {}
        self.cmd_clients = {}

        for ns in self.drones:
            self.states[ns] = State()
            self.altitudes[ns] = 0.0

            self.create_subscription(
                State,
                f'/{ns}/state',
                lambda msg, n=ns: self.state_cb(msg, n),
                10
            )

            self.create_subscription(
                PoseStamped,
                f'/{ns}/local_position/pose',
                lambda msg, n=ns: self.pose_cb(msg, n),
                10
            )

            self.mode_clients[ns] = self.create_client(
                SetMode,
                f'/{ns}/set_mode'
            )

            self.arm_clients[ns] = self.create_client(
                CommandBool,
                f'/{ns}/cmd/arming'
            )

            self.cmd_clients[ns] = self.create_client(
                CommandLong,
                f'/{ns}/cmd/command'
            )

        self.get_logger().info("🚁 SWARM TAKEOFF BAŞLATILIYOR")

        self.wait_for_connections()
        self.set_guided_all()
        self.arm_all()
        self.takeoff_sync(3.0)

    def state_cb(self, msg, ns):
        self.states[ns] = msg

    def pose_cb(self, msg, ns):
        self.altitudes[ns] = msg.pose.position.z

    def wait_for_connections(self):
        self.get_logger().info("⏳ Bağlantılar bekleniyor...")
        while rclpy.ok():
            if all(self.states[n].connected for n in self.drones):
                self.get_logger().info("✅ Tüm dronelar bağlı")
                return
            rclpy.spin_once(self, timeout_sec=0.5)

    def set_guided_all(self):
        self.get_logger().info("🧭 GUIDED moda geçiliyor")
        for ns in self.drones:
            self.mode_clients[ns].wait_for_service()
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.mode_clients[ns].call_async(req)
        time.sleep(2)

    def arm_all(self):
        self.get_logger().info("🔐 ARM ediliyor")
        for ns in self.drones:
            self.arm_clients[ns].wait_for_service()
            req = CommandBool.Request()
            req.value = True
            self.arm_clients[ns].call_async(req)
        time.sleep(3)

    def send_takeoff(self, altitude):
        for ns in self.drones:
            req = CommandLong.Request()
            req.command = 22  # MAV_CMD_NAV_TAKEOFF
            req.param7 = altitude
            self.cmd_clients[ns].call_async(req)

    def takeoff_sync(self, altitude):
        self.get_logger().info(f"🚀 EŞ ZAMANLI TAKEOFF → {altitude} m")

        # ilk takeoff
        self.send_takeoff(altitude)

        start = time.time()
        timeout = 15.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            airborne = [self.altitudes[n] > 1.0 for n in self.drones]

            if all(airborne):
                self.get_logger().info("🎉 TÜM DRONELAR AYNI ANDA HAVADA")
                return

            # 5 sn geçtiyse tekrar takeoff gönder
            if time.time() - start > 5.0:
                self.get_logger().warn("🔁 Kalkmayan drone var → TAKEOFF tekrar")
                self.send_takeoff(altitude)
                start = time.time()

            if time.time() - start > timeout:
                self.get_logger().error("❌ TAKEOFF ZAMAN AŞIMI")
                return


def main(args=None):
    rclpy.init(args=args)
    node = SwarmTakeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




 


  
       

     










