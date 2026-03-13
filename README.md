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

---

## 🎯 Simülasyonu Sıfırdan Başlatma (Adım Adım)

### 1️⃣ Gerekli Bağımlılıkları Kontrol Et

```bash
# ROS2 Humble kurulu mu?
ros2 --version

# Gazebo Harmonic (gz-sim) kurulu mu?
gz sim --version

# ArduPilot SITL kurulu mu?
test -d ~/ardupilot && echo "✓ ArduPilot var" || echo "✗ ArduPilot yok"

# MAVROS kurulu mu?
ros2 pkg list | grep mavros
```

### 2️⃣ Workspace'i İndir ve Derle

```bash
# 1. Workspace oluştur
mkdir -p ~/gz_ws/src
cd ~/gz_ws/src

# 2. Ana paketi klonla (modeller dahil!)
git clone https://github.com/SeydaGulKOCAK/sampiyonluk.git my_swarm_pkg

# 3. swarm_msgs paketini klonla (mesaj tipleri)
# Not: Ekip arkadaşından swarm_msgs reposunu al veya create et
# git clone <swarm_msgs_repo_url> swarm_msgs

# 4. ardupilot_gazebo plugin'ini klonla (eğer yoksa)
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
# NOT: sudo make install YAPMA - workspace içinde kullanacağız

# 5. Workspace'i derle
cd ~/gz_ws
colcon build --symlink-install
source install/setup.bash
```

### 3️⃣ Ortam Değişkenlerini Ayarla

```bash
# Tek seferlik (her terminalde tekrarla)
source ~/gz_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$HOME/gz_ws/src/my_swarm_pkg/config/cyclonedds_localhost.xml"

# YA DA swarm_env.sh kullan (önerilen)
source ~/gz_ws/src/my_swarm_pkg/config/swarm_env.sh
```

### 4️⃣ Simülasyonu Başlat

#### 🚀 YOL 1: Tek Komutla Başlatma (Önerilen)

```bash
cd ~/gz_ws/src/my_swarm_pkg
bash start_swarm.sh
```

Bu script:
- Gazebo'yu açar
- 3 ArduPilot SITL başlatır
- 3 MAVROS node başlatır
- Tüm swarm node'larını başlatır

#### 🛠️ YOL 2: Manuel Başlatma (Debug için)

**Terminal 1 - Gazebo:**
```bash
source ~/gz_ws/src/my_swarm_pkg/config/swarm_env.sh
gz sim ~/gz_ws/src/my_swarm_pkg/worlds/world_base.sdf
```

**Terminal 2 - ArduPilot SITL (Drone 1):**
```bash
cd ~/ardupilot
sim_vehicle.py -v ArduCopter --model JSON -I0 --sysid 1 --out=udp:127.0.0.1:14550
```

**Terminal 3 - ArduPilot SITL (Drone 2):**
```bash
cd ~/ardupilot
sim_vehicle.py -v ArduCopter --model JSON -I1 --sysid 2 --out=udp:127.0.0.1:14560
```

**Terminal 4 - ArduPilot SITL (Drone 3):**
```bash
cd ~/ardupilot
sim_vehicle.py -v ArduCopter --model JSON -I2 --sysid 3 --out=udp:127.0.0.1:14570
```

**Terminal 5 - MAVROS (3 drone):**
```bash
source ~/gz_ws/install/setup.bash
source ~/gz_ws/src/my_swarm_pkg/config/swarm_env.sh
ros2 launch my_swarm_pkg swarm_mavros.launch.py
```

**Terminal 6 - Swarm Node'ları:**
```bash
source ~/gz_ws/install/setup.bash
source ~/gz_ws/src/my_swarm_pkg/config/swarm_env.sh
ros2 launch my_swarm_pkg swarm_full.launch.py --no-sitl
```

### 5️⃣ Sistemi Test Et

```bash
# ROS2 node'ları kontrol et
ros2 node list

# MAVROS bağlantıları kontrol et
ros2 topic echo /drone1/mavros/state --once
ros2 topic echo /drone2/mavros/state --once
ros2 topic echo /drone3/mavros/state --once

# Swarm topic'leri kontrol et
ros2 topic list | grep swarm
```

### 6️⃣ Görev Başlatma Örnekleri

#### Görev 1: Otonom Kalkış + QR Tarama
```bash
# Mission FSM node'u çalışıyorsa, terminal dashboard'dan:
# 1 tuşuna bas → ARM
# 2 tuşuna bas → TAKEOFF
# 3 tuşuna bas → START_MISSION
```

#### Görev 2: Joystick Kontrolü
```bash
# Teleop ile başlat
ros2 launch my_swarm_pkg swarm_full.launch.py with_teleop:=true

# Klavye kontrolleri:
# W/S: İleri/Geri
# A/D: Sol/Sağ
# Q/E: Yukarı/Aşağı
# Boşluk: Acil durdur
```

### 7️⃣ Simülasyonu Durdurma

```bash
# start_swarm.sh kullandıysan:
bash ~/gz_ws/src/my_swarm_pkg/start_swarm.sh --stop

# Manuel durdurma:
pkill -9 gz
pkill -9 arducopter
pkill -9 -f "ros2.*mavros"
```

---

## 🐛 Sorun Giderme

### "models bulunamadı" hatası
```bash
# Modellerin doğru yerde olduğunu kontrol et
ls -la ~/gz_ws/src/my_swarm_pkg/models/

# GZ_SIM_RESOURCE_PATH ayarla
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/my_swarm_pkg/models:$GZ_SIM_RESOURCE_PATH
```

### MAVROS "FCU: not connected" hatası
```bash
# ArduPilot SITL portlarını kontrol et
ss -tuln | grep -E "14550|14560|14570"

# SITL log'larını kontrol et
tail -f /tmp/ArduCopter.log
```

### Drone'lar Gazebo'da görünmüyor
```bash
# Gazebo model listesini kontrol et
gz model --list

# World dosyasını kontrol et
gz sim ~/gz_ws/src/my_swarm_pkg/worlds/world_base.sdf -v 4
```

---

## 🚁 Hızlı Başlangıç — 3 Drone Aynı Anda Uçurma

Simülasyonda 3 drone'u **aynı anda** kaldırmak için aşağıdaki adımları izleyin.

### Gereksinimler

| Yazılım | Versiyon |
|---------|---------|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble |
| MAVROS | 2.x (`ros-humble-mavros`) |
| ArduPilot SITL | ArduCopter V4.7+ |
| Gazebo | gz-sim (Harmonic veya Garden) |
| CycloneDDS | `ros-humble-rmw-cyclonedds-cpp` |

### Adım 1 — Sistemi Başlat (Gazebo + SITL + MAVROS)

```bash
cd ~/sampiyonluk
bash start_3drone_fixed.sh
```

Bu script sırasıyla şunları açar:
1. **Gazebo** — `world_base.sdf` dünyasını `--render-engine ogre` ile başlatır
2. **3× ArduPilot SITL** — UDP çıkış portları: `14550`, `14560`, `14570`
3. **3× MAVROS** — Her biri ilgili UDP portundan veri alır (plugin whitelist ile)

> ⏳ Tüm sistemin hazır olması ~30 saniye sürer.

### Adım 2 — Bağlantıyı Doğrula

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1

# ros2 daemon'u sıfırla (ilk seferde gerekli)
ros2 daemon stop && ros2 daemon start

# 3 drone'un bağlı olduğunu doğrula
for d in drone1 drone2 drone3; do
  echo -n "$d: "
  ros2 topic echo /$d/state --once 2>/dev/null | grep connected
done
# Çıktı: connected: true (3 kez)
```

### Adım 3 — 3 Drone'u Aynı Anda Kaldır 🚀

```bash
python3 ~/sampiyonluk/takeoff_3drone.py
```

Script şu adımları otomatik yapar:
1. ✅ 3/3 bağlantı kontrolü
2. ✅ GUIDED mod geçişi (3'ü birden)
3. 🔥 ARM (3'ü birden)
4. 🚀 TAKEOFF — CommandTOL ile eşzamanlı kalkış
5. 📍 Hover — Havada pozisyon takibi (Ctrl+C ile çık)

### Adım 4 — İndirme

```bash
# 3 drone'u aynı anda indir
for d in drone1 drone2 drone3; do
  ros2 service call /$d/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'LAND'}"
done
```

### Adım 5 — Sistemi Kapat

```bash
pkill -9 -f "gz sim"
pkill -9 -f "arducopter"
pkill -9 -f "mavros_node"
```

### ⚠️ Bilinen Sorunlar ve Çözümler

| Sorun | Çözüm |
|-------|-------|
| Gazebo ogre2 ile çöküyor | `--render-engine ogre` kullanılıyor (otomatik) |
| MAVROS "invalid allocator" crash | `config/mavros_plugins.yaml` ile plugin whitelist (otomatik) |
| `ros2 topic list` boş | `ros2 daemon stop && ros2 daemon start` |
| MAVROS topic'leri görünmüyor | `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` ayarla |
| ARM başarısız | Drone'un GUIDED modda olduğundan emin ol |

### 📂 Bu Bölümde Kullanılan Dosyalar

| Dosya | Açıklama |
|-------|---------|
| `start_3drone_fixed.sh` | Gazebo + 3 SITL + 3 MAVROS başlatıcı |
| `takeoff_3drone.py` | 3 drone eşzamanlı ARM + Takeoff scripti |
| `config/mavros_plugins.yaml` | MAVROS plugin whitelist (crash önleyici) |

### 🔧 MAVROS 2.x Servis Haritası

```
ARM      → /droneX/mavros/arming    (CommandBool)
Takeoff  → /droneX/mavros/takeoff   (CommandTOL)
Set Mode → /droneX/set_mode         (SetMode)
Command  → /droneX/mavros/command   (CommandLong)
State    → /droneX/state            (State)
```

---

