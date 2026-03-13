#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandLong, CommandTOL
from geometry_msgs.msg import PoseStamped

# State topic için QoS - MAVROS ile uyumlu
state_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Sensor/Pose topic için QoS (BEST_EFFORT)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)



class SwarmTakeoff(Node):
    def __init__(self):
        super().__init__('swarm_takeoff')
        self.drones = ['drone1', 'drone2', 'drone3']
        self.states = {d: None for d in self.drones}
        self.arm_clients = {}
        self.mode_clients = {}
        self.takeoff_clients = {}

        for drone in self.drones:
            ns = f'/{drone}'
            self.create_subscription(State, f'{ns}/state', self.state_cb_factory(drone), qos_profile=state_qos)
            self.arm_clients[drone] = self.create_client(CommandBool, f'{ns}/cmd/arming')
            self.mode_clients[drone] = self.create_client(SetMode, f'{ns}/set_mode')
            self.takeoff_clients[drone] = self.create_client(CommandTOL, f'{ns}/cmd/takeoff')

        # Kalkış adımlarını başlat (daha sık kontrol)
        self.timer = self.create_timer(0.2, self.takeoff_sequence)
        self.step = 0
        self.last_action_time = time.time()
        self.start_time = time.time()
        self.timeout_sec = 60  # Her adım için maksimum bekleme süresi (saniye)


    def state_cb_factory(self, drone):
        def cb(msg):
            self.states[drone] = msg
        return cb

    def takeoff_sequence(self):
        now = time.time()
        elapsed = int(now - self.start_time)
        # Her döngüde tüm droneların state'ini logla
        for d in self.drones:
            s = self.states[d]
            if s:
                self.get_logger().info(f"{d}: connected={s.connected}, mode={s.mode}, armed={s.armed}")
            else:
                self.get_logger().info(f"{d}: state yok")

        # 0. ADIM: Tüm dronelar bağlı mı?
        if self.step == 0:
            connected = [d for d in self.drones if self.states[d] and getattr(self.states[d], 'connected', False)]
            if len(connected) == len(self.drones):
                self.get_logger().info(f"✅ Tüm dronelar bağlı! → GUIDED moda geçiliyor")
                for d in self.drones:
                    req = SetMode.Request(custom_mode="GUIDED")
                    self.mode_clients[d].call_async(req)
                self.step = 1
                self.last_action_time = now
                self.start_time = now
            else:
                self.get_logger().info(f"⏳ Bağlantı bekleniyor: {connected} bağlı / {self.drones}")
                if elapsed > self.timeout_sec:
                    self.get_logger().error("❌ Zaman aşımı: Tüm dronelar bağlanamadı! Simülasyon/MAVROS ayarlarını ve bağlantıları kontrol et.")
                    self.step = 99
        # 1. ADIM: Tüm dronelar GUIDED modda mı?
        elif self.step == 1:
            guided = [d for d in self.drones if self.states[d] and getattr(self.states[d], 'mode', '') == "GUIDED"]
            if len(guided) == len(self.drones):
                self.get_logger().info(f"✅ Tüm dronelar GUIDED! → ARM ediliyor")
                for d in self.drones:
                    req = CommandBool.Request(value=True)
                    self.arm_clients[d].call_async(req)
                self.step = 2
                self.last_action_time = now
                self.start_time = now
            elif now - self.last_action_time > 3.0:
                # Tekrar GUIDED komutu gönder
                for d in self.drones:
                    req = SetMode.Request(custom_mode="GUIDED")
                    self.mode_clients[d].call_async(req)
                self.get_logger().info(f"🔁 GUIDED komutu tekrar gönderildi.")
                self.last_action_time = now
            if elapsed > self.timeout_sec:
                self.get_logger().error("❌ Zaman aşımı: Tüm dronelar GUIDED moda geçemedi! Pre-arm check, GPS lock, home position gibi gereklilikleri ve simülasyon ayarlarını kontrol et.")
                self.step = 99
        # 2. ADIM: Tüm dronelar ARM oldu mu?
        elif self.step == 2:
            armed = [d for d in self.drones if self.states[d] and getattr(self.states[d], 'armed', False)]
            if len(armed) == len(self.drones):
                self.get_logger().info(f"✅ Tüm dronelar ARM! → Kalkış başlıyor")
                for d in self.drones:
                    req = CommandTOL.Request(altitude=3.0, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
                    self.takeoff_clients[d].call_async(req)
                self.step = 3
                self.last_action_time = now
                self.start_time = now
            elif now - self.last_action_time > 3.0:
                # Tekrar ARM komutu gönder
                for d in self.drones:
                    req = CommandBool.Request(value=True)
                    self.arm_clients[d].call_async(req)
                self.get_logger().info(f"🔁 ARM komutu tekrar gönderildi.")
                self.last_action_time = now
            if elapsed > self.timeout_sec:
                self.get_logger().error("❌ Zaman aşımı: Tüm dronelar ARM olamadı! Pre-arm check, GPS lock, home position gibi gereklilikleri ve simülasyon ayarlarını kontrol et.")
                self.step = 99
        # 3. ADIM: Kalkış sonrası izleme (isteğe bağlı daha fazla kontrol eklenebilir)
        elif self.step == 3:
            self.get_logger().info(f"🎉 Kalkış komutları gönderildi! 3 drone aynı anda havalanıyor.")
            self.step = 4
        # 4. ADIM: İzleme veya sonlandırma
        elif self.step == 99:
            self.get_logger().error("⛔ Görev sonlandırıldı. Lütfen yukarıdaki hata mesajlarını incele!")
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SwarmTakeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()




 


  
       

     










