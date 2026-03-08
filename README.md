# 🏆 Şampiyonluk — TEKNOFEST 2026 Sürü İHA Yazılımı

> **TEKNOFEST 2026 Sürü İHA Yarışması** | ÖTR Deadline: 27 Mart 2026

ROS2 Humble + Gazebo gz-sim + ArduCopter SITL tabanlı 3 drone'lu otonom sürü sistemi.

---

## 📁 Paket Yapısı

```
my_swarm_pkg/
├── my_swarm_pkg/              # Python node'ları
│   ├── drone_interface.py     # MAVROS köprüsü (arm/takeoff/setpoint)
│   ├── local_fsm.py           # Drone durum makinesi (STANDBY→FLYING→…)
│   ├── intent_coordinator.py  # Bully lider seçimi + QR görev dağıtımı
│   ├── formation_controller.py# Virtual Structure formasyon (OKBASI/V/CIZGI)
│   ├── qr_perception.py       # ArUco/renk QR algılama (Şekil 2 JSON)
│   ├── swarm_takeoff.py       # Eşzamanlı kalkış koordinasyonu
│   ├── collision_avoidance.py # APF tabanlı çarpışma önleme (50 Hz)
│   ├── safety_monitor.py      # Batarya + geofence + heartbeat izleme
│   ├── mission_fsm.py         # GCS durum makinesi + terminal dashboard
│   ├── waypoint_navigator.py  # QR→QR waypoint takibi (10 Hz)
│   └── swarm_teleop.py        # Klavye/joystick Görev-2 kontrolü
├── launch/
│   ├── swarm_full.launch.py   # TAM SİSTEM başlatma (Gazebo+SITL+MAVROS+nodes)
│   ├── swarm_ardupilot.launch.py
│   └── swarm_mavros.launch.py
├── worlds/
│   └── world_base.sdf         # Gazebo dünyası (QR markerlar, home noktaları)
├── config/
│   ├── qr_map.yaml            # 6 QR waypoint koordinatları
│   ├── cyclonedds_localhost.xml # DDS loopback config
│   └── swarm_env.sh           # Ortam değişkenleri (source et)
├── start_swarm.sh             # Tek komutla sistem başlatma scripti
├── package.xml
└── setup.py
```

---

## 🛸 Node'lar ve Görevleri

| Node | Frekans | Görev |
|---|---|---|
| `drone_interface` | olay tabanlı | MAVROS ↔ ROS2 köprüsü |
| `local_fsm` | 10 Hz | Drone durum makinesi |
| `intent_coordinator` | 2 Hz | Bully lider seçimi, QR dağıtımı |
| `formation_controller` | 20 Hz | Virtual Structure setpoint üretimi |
| `qr_perception` | kamera fps | ArUco + renk QR algılama |
| `swarm_takeoff` | olay tabanlı | Eşzamanlı kalkış |
| `collision_avoidance` | 50 Hz | APF çarpışma önleme |
| `safety_monitor` | 5 Hz | Batarya + geofence + heartbeat |
| `mission_fsm` | 1 Hz (dashboard) | GCS operatör arayüzü |
| `waypoint_navigator` | 10 Hz | QR waypoint takibi |
| `swarm_teleop` | 20 Hz | Görev-2 klavye/joystick kontrolü |

---

## 🔧 Kurulum

### Gereksinimler
- Ubuntu 22.04
- ROS2 Humble
- Gazebo Harmonic (gz-sim 8)
- ArduPilot + ardupilot_gazebo plugin
- MAVROS (`ros-humble-mavros`)

### Kurulum Adımları

```bash
# 1. Workspace oluştur
mkdir -p ~/gz_ws/src && cd ~/gz_ws/src

# 2. Repoyu klonla
git clone https://github.com/SeydaGulKOCAK/sampiyonluk.git my_swarm_pkg

# 3. swarm_msgs bağımlılığını da klonla (aynı ekip reposundan)
# git clone <swarm_msgs_repo> swarm_msgs

# 4. Build
cd ~/gz_ws
colcon build --packages-select swarm_msgs my_swarm_pkg
source install/setup.bash
```

---

## 🚀 Çalıştırma

### Hızlı Başlatma (Önerilen)
```bash
# Ortamı yükle
source ~/gz_ws/src/my_swarm_pkg/config/swarm_env.sh

# Tam sistemi başlat (Gazebo + SITL + MAVROS + tüm node'lar)
bash ~/gz_ws/src/my_swarm_pkg/start_swarm.sh

# Sadece ROS2 node'ları (SITL/MAVROS dışarıdan çalışıyorsa)
bash ~/gz_ws/src/my_swarm_pkg/start_swarm.sh --no-sitl
```

### Manuel Başlatma (launch dosyası ile)
```bash
source ~/gz_ws/src/my_swarm_pkg/config/swarm_env.sh

# Tam sistem
ros2 launch my_swarm_pkg swarm_full.launch.py

# Görev-2 joystick ile
ros2 launch my_swarm_pkg swarm_full.launch.py with_teleop:=true

# Debug modu
ros2 launch my_swarm_pkg swarm_full.launch.py log_level:=debug
```

### Sistemi Durdurma
```bash
bash ~/gz_ws/src/my_swarm_pkg/start_swarm.sh --stop
```

---

## 🌐 Ağ / DDS Ayarı

Wi-Fi olmadan çalışıyorsan (sadece loopback):
```bash
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file:///home/seyda/gz_ws/src/my_swarm_pkg/config/cyclonedds_localhost.xml"
```
Bu ayarlar `swarm_env.sh` içinde otomatik yapılır.

---

## 🗺️ Dünya Koordinatları

- **Alan**: x∈[0, 120] m, y∈[0, 90] m
- **QR Waypoint rotası**: (10,20) → (10,50) → (55,75) → (100,50) → (100,20) → (55,5)
- **Home noktaları**: Drone1=(0,0), Drone2=(4,0), Drone3=(8,0)
- **Geofence**: x∈[-5,125], y∈[-5,95], z∈[0.5,60] m

---

## 📡 Önemli Topic'ler

```
/swarm/intent          # Aktif görev niyeti (lider yayınlar)
/swarm/virtual_leader  # Formasyon merkez noktası
/swarm/teleop_cmd      # Joystick komutları (Görev-2)
/safety/event          # Güvenlik olayları (BATTERY_CRITICAL, GEOFENCE_BREACH)
/{ns}/local_state      # Drone durum makinesi çıktısı
/{ns}/battery_pct      # Batarya yüzdesi (simüle)
/qr/result             # QR okuma sonucu (JSON)
```

---

## 👥 Ekip

| Kişi | Görev |
|---|---|
| Seyda | Yazılım mimarisi, ROS2 node'ları |
| Rukiye | swarm_teleop, test |
| ... | ... |

---

## ⚠️ Bilinen Sorunlar

- `sim_vehicle.py --no-mavproxy` flag'i eski ArduPilot sürümlerinde desteklenmeyebilir → kaldır
- Gazebo ilk açılışta model yükleme ~5-8 saniye sürebilir
- `ROS_LOCALHOST_ONLY=1` tüm terminallerde set edilmeli

---

*TEKNOFEST 2026 — Sürü İHA Yarışması | 8 Mart 2026*
