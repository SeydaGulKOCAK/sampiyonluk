# 🏆 TEKNOFEST 2026 — SWARM SIMULASYON SİSTEMİ

**Sürü İHA (UAV) Otonom Misyon — 3 Drone Senkron Operasyon**

---

## 📚 DOKÜMANTASYON KAPAKLERİ

Bu klasörde bulunacaklar:

### 🎯 **BAŞLANGIÇ NOKTASI**
- **[QUICK_START_YARIŞMA.md](QUICK_START_YARIŞMA.md)** — En hızlı başlama (5 adım)
- **[start_swarm_competition.sh](start_swarm_competition.sh)** — Tek komutu çalıştır: sistem başlat

### 📋 **YARISMA GÜNÜ**
- **[YARIŞMA_GUNU_PROTOKOLÜ.md](YARIŞMA_GUNU_PROTOKOLÜ.md)** — Detaylı rehber
  - Hazırlık kontrol listesi
  - Kronolojik timeline
  - Jüri koordinatlarını girme
  - Sorun çözüm
  - Başarı kriterleri

### 🔧 **TEKNIK ARAÇLAR**
- **[src/my_swarm_pkg/my_swarm_pkg/setup_qr_coordinates.py](src/my_swarm_pkg/my_swarm_pkg/setup_qr_coordinates.py)** — Koordinat yükleme scripti
  - Etkileşimli CLI
  - SetQRMap service client
  - Jüri koordinatlarını runtime'da gönder

### 🎮 **LAUNCH VE KONFIGÜRASYON**
- **[src/my_swarm_pkg/launch/swarm_competition.launch.py](src/my_swarm_pkg/launch/swarm_competition.launch.py)** — Main launch dosyası
  - 3 drone senkron başlatma (SITL delays: 0s, 2s, 4s)
  - MAVROS otomatik bağlantısı
  - Gazebo world spawning
  - GCS dashboard (xterm)
  
- **[src/my_swarm_pkg/config/swarm_params.yaml](src/my_swarm_pkg/config/swarm_params.yaml)** — Configuration
  - Drone home pozisyonları
  - Formation offsets
  - Geofence sınırları
  - QR detection parametreleri

---

## ⚡ EN HIZLI BAŞLAMA (3 KOMUt)

```bash
# 1️⃣  ROS2 ortamını hazırla
cd ~/gz_ws && . install/setup.bash

# 2️⃣  Sistemi başlat (Gazebo + SITL + MAVROS + Dashboard)
ros2 launch my_swarm_pkg swarm_competition.launch.py

# 3️⃣  xterm dashboard'da:
# [m] → Jüri koordinatlarını gir
# [s] → TASK1 başlat (3 drone senkron kalkar)
```

**Çıktı:** ~15 saniyede hazır, 3 drone aynı anda havalanıyor ✈️

---

## 🎯 SİSTEM ARKİTEKTÜRÜ

```
┌─────────────────────────────────────────────────────────────┐
│                    GAZEBO (Simulator)                        │
│         (world_base.sdf + 3 drone physics models)           │
└────────────────┬────────────────────────────────────────────┘
                 │ UDP ports: 14550, 14560, 14570
┌────────────────▼────────────────────────────────────────────┐
│            ARDUPILOT SITL (3 instances)                      │
│         drone1(sysid=1), drone2(2), drone3(3)               │
└────────────────┬────────────────────────────────────────────┘
                 │ MAVLink over UDP
┌────────────────▼────────────────────────────────────────────┐
│         MAVROS (3 instances, /drone1 → /drone3)             │
│    (sensor bridge, setpoint publisher, parameter manager)   │
└────────────────┬────────────────────────────────────────────┘
                 │ ROS2 topics & services
┌────────────────▼────────────────────────────────────────────┐
│              SWARM CONTROL LAYER                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Local FSM (drone_interface, local_fsm)              │   │
│  │   • Drone state management (STANDBY→FLYING)         │   │
│  │   • Service calls to MAVROS                         │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Global Coordination (intent_coordinator)            │   │
│  │   • Leader election (Bully algorithm)               │   │
│  │   • Intent broadcasting (/swarm/intent)             │   │
│  │   • QR map validation (/swarm/qr_map_ready)        │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Formation Control (formation_controller)            │   │
│  │   • Virtual Structure following                     │   │
│  │   • Relative positioning (4m, 8m offsets)          │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Perception & Navigation (GCS)                      │   │
│  │   • QR detection (radius-based 5m)                 │   │
│  │   • SetQRMap service (/swarm/set_qr_map)           │   │
│  │   • Waypoint navigation                             │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Safety (collision_avoidance, safety_monitor)       │   │
│  │   • Geofence enforcement                            │   │
│  │   • Collision prediction                            │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

---

## 📊 OPERASYON SEKANSSI (TASK1)

```
T+0s  │ [m] tuşu: Jüri koordinatlarını gir (6 QR konumu)
      │
T+5s  │ [s] tuşu: TASK1 başlat
      │
T+6s  │ ✈️ TAKEOFF FAZII
      │ drone1, drone2, drone3 ARMING → FLYING
      │ All: Z = QR1_height (senkron)
      │
T+10s │ 📡 NAVIGATION BAŞLANGICI
      │ Leader (drone1) QR1'e gidiyor
      │ Followers: Virtual Structure formation oluştur
      │
T+20s │ 🎯 QR DETECTION
      │ drone1 (5m range): QR1 algılandı
      │ → Rota: QR1 → QR2
      │
T+30s │ QR2 algılandı → QR3
      │ ... (QR6'ya kadar devam)
      │
T+50s │ 🏠 HOME DÖNÜŞÜ
      │ drone1: (0, 0, 0) iniş
      │ drone2, drone3: formation iniş
      │
T+60s │ ✅ MISYON BİTTİ
      │ Tüm drone'ler LANDED, motorlar off
```

---

## 🔑 TEMEL DOSYALAR

| Dosya | Amaç | Değiştir mi? |
|-------|------|--------------|
| `launch/swarm_competition.launch.py` | Sistem başlatma | ⚠️ Setup delays gerekirse |
| `config/swarm_params.yaml` | Drone pozisyonları, formation | ⚠️ Formation değişirse |
| `my_swarm_pkg/intent_coordinator.py` | Leader seçimi, QR dağıtımı | ❌ Ciddiyetli logic |
| `my_swarm_pkg/mission_fsm.py` | Dashboard, TASK trigger | ✅ UI komutları opsiyonel |
| `my_swarm_pkg/qr_perception.py` | SetQRMap service | ✅ Detection parametreleri |

---

## 🚀 TÜRKÇE KOMUT REFERANSı

**Mission FSM Dashboard (xterm):**

```
[m] veya [map]  → Harita Kur (Jüri Koordinatları)
[s] veya [start]  → TASK1 Başlat (Senkron Kalkış)
[p] veya [pause]  → Askıya Al (HOVER)
[r] veya [resume] → Devam Et (Rota)
[h] veya [home]   → Eve Döndür (HOME)
[q] veya [quit]   → Çık (Sistem Kapat)
[?] veya [help]   → Yardım (Komut Listesi)
```

---

## ⚙️ TEMEL AYARLAMALAR

### Drone Başlangıç Konumları
```yaml
# swarm_params.yaml
drone1_home: [0.0, 0.0, 0.0]      # Lider orta
drone2_home: [4.0, 0.0, 0.0]      # Sağında 4m
drone3_home: [8.0, 0.0, 0.0]      # Sağında 8m
```

### Formation Offsets (Lider'e göre)
```yaml
drone1_offset: [0, 0, 0]    # Lider
drone2_offset: [4, 0, 0]    # Yan yana
drone3_offset: [8, 0, 0]    # Çizgi
```

### Geofence Alanı (Güvenlik)
```yaml
x_range: [-5, 125] m     # X ekseni
y_range: [-5, 95] m      # Y ekseni
z_range: [0.5, 60] m     # Yükseklik
```

---

## 📞 ACİL DURUM PROTOKOLÜ

| Durum | Komut |
|-------|-------|
| **Sistem kilitlendi** | `Ctrl+C` (terminal'de) |
| **Drone'ler askıda kalsın** | `[p]` (dashboard'da) |
| **Eve dönüş** | `[h]` (dashboard'da) |
| **Tüm işlemi kapat** | `Ctrl+C` + Gazebo ✕ |

---

## 🎓 İLERİ AYARLAMALAR

### Başlangıç Zamanlamasını Değiştir
```python
# swarm_competition.launch.py
# SITL spawn delays (saniye)
TimerAction(period=0.0, actions=[...])  # drone1: hemen
TimerAction(period=2.0, actions=[...])  # drone2: 2s sonra
TimerAction(period=4.0, actions=[...])  # drone3: 4s sonra
```

### MAVROS Bağlantı Gecikmesi
```python
# ~280 satır
delay=9.0  # MAVROS için bekleme (SITL'den sonra)
# Drone2/3 çok geç kalkarsa, 11.0'a artır
```

---

## 📈 PERFORMANS BEKLENTILERI

- ✅ **Sistem başlatma:** ~15 saniye
- ✅ **Gazebo rendering:** 30 FPS (3 drone)
- ✅ **MAVROS bağlantı:** ~8 saniye
- ✅ **Senkron kalkış:** <1 saniye fark
- ✅ **Formation oluşma:** <3 saniye
- ✅ **QR detection:** 5m radius'ta
- ✅ **Misyon süresi:** 5-8 dakika (waypoint'lere bağlı)

---

## 🔗 İLİŞKİLİ DOSYALAR

```
📦 sampiyonluk/
├── 📘 README.md (genel)
├── ⚡ QUICK_START_YARIŞMA.md (hızlı)
├── 📋 YARIŞMA_GUNU_PROTOKOLÜ.md (detaylı)
├── 🔧 start_swarm_competition.sh (otomatik başlatma)
│
└── src/
    ├── swarm_msgs/
    │   └── srv/SetQRMap.srv (jüri koordinat servisi)
    │
    └── my_swarm_pkg/
        ├── launch/
        │   └── swarm_competition.launch.py (main)
        ├── config/
        │   └── swarm_params.yaml (ayarlar)
        └── my_swarm_pkg/
            ├── drone_interface.py
            ├── local_fsm.py
            ├── intent_coordinator.py
            ├── formation_controller.py
            ├── collision_avoidance.py
            ├── safety_monitor.py
            ├── qr_perception.py (SetQRMap service)
            ├── waypoint_navigator.py
            ├── mission_fsm.py (dashboard)
            └── setup_qr_coordinates.py (jüri CLI)
```

---

## 🏆 BAŞARILAR DİLEYİZ!

**İnşaallah TEKNOFEST 2026'de birincilik! 🚀**

---

**Doküman Versiyon:** v2.0  
**Son Güncelleme:** Ocak 2026  
**Bakım:** Seyda & Takım
