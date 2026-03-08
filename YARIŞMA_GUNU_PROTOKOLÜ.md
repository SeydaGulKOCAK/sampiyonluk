# 🏆 TEKNOFEST 2026 — YARIŞMA GÜNÜ PROTOKOLÜ

**İnşaallah başarılı geçer! 🙏**

---

## 📋 HAZIRLIK KONTROL LİSTESİ (Yarışma Gününden 30 dakika önce)

- [ ] Bilgisayar açılmış, ağ bağlı
- [ ] Gazebo, ArduPilot SITL, MAVROS başlamaya hazır
- [ ] ROS2 ortamı kurulmuş: `. ~/gz_ws/install/setup.bash`
- [ ] Jüriden QR koordinatları öğrenilmiş (6 waypoint + home)

---

## ⏱️ YARIŞMA GÜNÜNÜN KRONOLOJISI

### **T-5 dakika: Sistem Başlatma** 🚀

```bash
# Terminal 1: Swarm launch (Gazebo + SITL + MAVROS + nodes)
cd ~/gz_ws
colcon build --packages-select swarm_msgs my_swarm_pkg  # (yapıldıysa skip)
ros2 launch my_swarm_pkg swarm_competition.launch.py
```

**Çıktı beklentileri:**
```
[INFO] ✅ Gazebo başlatıldı (world_base.sdf yükleniyor)
[INFO] ⏳ SITL drone1 başlıyor (port 14550)
[INFO] ⏳ SITL drone2 başlıyor (port 14560) — 2 sn sonra
[INFO] ⏳ SITL drone3 başlıyor (port 14570) — 4 sn sonra
[INFO] ⏳ MAVROS drone1 başlıyor — 8 sn sonra
[INFO] ⏳ MAVROS drone2 başlıyor
[INFO] ⏳ MAVROS drone3 başlıyor
[INFO] ✅ Tüm drone'ler bağlantı sağladı
[INFO] 🎮 Mission FSM xterm penceresi açılacak — Kontrol paneli burada!
```

⏱️ **Total başlama süresi:** ~15 saniye

**Gazebo'da görmek için:** 3 drone, 0,0 ile 8,0 arasında sıralanmış (x ekseni boyunca)

---

### **T-3 dakika: Jüri Koordinatlarını Yükleme** 📍

Mission FSM xterm penceresi açıldığında şunu göreceksiniz:
```
╔════════════════════════════════════════════════════╗
║    MISSION FSM DASHBOARD — Swarm Kontrol Paneli   ║
║    Press [m] to setup QR map (jüri koordinatları) ║
║    Press [s] to start TASK1 (autonomous mission)  ║
║    Press [q] to quit                              ║
╚════════════════════════════════════════════════════╝
```

**ADIM 1: `m` tuşuna basın**
```
Press: m
```

**ADIM 2: Jüriden aldığınız koordinatları girin**

Jüri size verecek:
```
QR1: X1, Y1, Z1
QR2: X2, Y2, Z2
...
QR6: X6, Y6, Z6
```

Giriş şekli:
```
QR1: 10.0,20.0,15.0
QR2: 10.0,50.0,15.0
QR3: 55.0,75.0,20.0
QR4: 55.0,45.0,18.0
QR5: 100.0,20.0,15.0
QR6: 100.0,80.0,20.0
[ENTER] (boş satır)
```

**Sistem yanıtı:**
```
✅ JÜRİ HARİTASI BAŞARIYLA YÜKLENDİ!
   6 waypoint kaydedildi, rota: 1→2→3→4→5→6→HOME

🚀 Şimdi [s] tuşuna basın (3 drone SENKRON kalkar)
```

---

### **T-1 dakika: Otonom Misyon Başlama** 🎯

**ADIM 3: `s` tuşuna basın**
```
Press: s
```

**TASK1 başlıyor:**

```
[INFO] ✅ intent_coordinator: TASK1 başlandı!
[INFO] ⏳ drone1: STANDBY → IDLE
[INFO] ⏳ drone2: STANDBY → IDLE
[INFO] ⏳ drone3: STANDBY → IDLE

[INFO] ⏳ drone1: IDLE → ARMING
[INFO] ⏳ drone2: IDLE → ARMING  ← drone2 drone1 ile yaklaşık aynı anda
[INFO] ⏳ drone3: IDLE → ARMING  ← drone3 da senkron

[INFO] ✅ drone1: ARMED (motorlar döner, propeller kontrol altında)
[INFO] ✅ drone2: ARMED
[INFO] ✅ drone3: ARMED

[INFO] ⏳ drone1: ARMING → FLYING (kalkış talimatı veriliyor)
[INFO] ⏳ drone2: ARMING → FLYING
[INFO] ⏳ drone3: ARMING → FLYING

[INFO] ✅ drone1: Taking off to Z=15.0 m (QR1 yüksekliği)
[INFO] ✅ drone2: Taking off (drone1'in 4m gerisinde, sağında)
[INFO] ✅ drone3: Taking off (drone1'in 8m gerisinde, çizgi oluşur)

[INFO] 📡 Waypoint gönderiliyor: QR1 (10.0, 20.0, 15.0)
[INFO] 📡 drone1: Rota başladı → QR1'e doğru
[INFO] 📡 drone2: Takip ediyor (formation control)
[INFO] 📡 drone3: Takip ediyor (formation control)
```

---

## 👀 GAZEBO VİZÜEL DOĞRULAMASI

Simulasyon sırasında Gazebo'da göreceksiniz:

1. **Kalkış fazı (İlk 5 saniye):**
   - 3 drone yerden yükselmek için motorları hızlandırıyor
   - Z ekseninde (yüksek) hareket ediyor
   - X, Y henüz sabit kalıyor

2. **Rota başlangıcı (5-30 saniye):**
   - Drone1 ön tarafa doğru eğiliyor (QR1'e gidiyor)
   - Drone2 ve Drone3 takip ediyor (formation)
   - Gazebo'da 3 drone arasında görünen mesafe sabitleniyor

3. **QR algılama (Her waypoint'e 100m'ye yaklaştığında):**
   - "QR detected" mesajı xterm'de görünür
   - Rota otomatik olarak bir sonraki QR'ya geçer

4. **Son waypoint'ten sonra:**
   - Tüm drone'ler HOME (0,0,0) konumuna dönerken
   - Iniş fazında

---

## 🔴 KRİZİS YÖNETİMİ (Sorun Çıkarse)

### **Sorun: Sistem başlamıyor**

```bash
# Terminal'de şunu çalıştırın (ROS2 çevre değişkenleri kontrol)
env | grep ROS
env | grep CYCLONEDDS
```

**Çözüm:**
```bash
# Terminali kapat ve bu komutla yeniden aç
cd ~/gz_ws
. install/setup.bash
echo "Kontrol: $ROS_LOCALHOST_ONLY = 1 olmalı"
ros2 launch my_swarm_pkg swarm_competition.launch.py
```

---

### **Sorun: Gazebo başlamıyor ama SITL başlıyor**

```
[ERROR] Gazebo spawn başarısız
```

**Çözüm:**
```bash
# Gazebo'nun bağımsız başlatılıp başlatılamadığını kontrol et
gz sim worlds/teknofest_clean_arena.sdf
# (Gazebo penceresi açılmalı ve simulator'da 3 drone görülmeli)
```

---

### **Sorun: MAVROS bağlanmıyor (bkz: "SYTEM NOT READY")**

```
[WARN] drone1: MAVROS service not ready (retrying)
```

**Neden:** SITL'ler başladı ama MAVROS'un MAVLink discovery'si zaman alıyor

**Çözüm:** 15-20 saniye bekleyin. Sistem otomatik retry yapıyor. Çoğu zaman kendini düzeltiyor.

Eğer 30 saniyeden sonra hala "SYSTEM NOT READY" ise:
```bash
# Yeni terminal aç
ros2 service list | grep set_mode
# Çıktı: /drone1/mavros/set_mode
# Görülüyorsa MAVROS bağlantı hazır demektir.
```

---

### **Sorun: Mission FSM xterm penceresi açılmıyor**

```
[ERROR] xterm not found
```

**Çözüm:** xterm'i yükle
```bash
sudo apt install xterm
```

Eğer xterm'i yüklemek istemiyorsanız, Mission FSM'i farklı terminal'de başlatın:
```bash
# Yeni terminal aç
cd ~/gz_ws
. install/setup.bash
ros2 run my_swarm_pkg mission_fsm
# Dashboard açılacak (aynı şekilde [m], [s], [q] tuşları çalışır)
```

---

### **Sorun: Drone TASK1 başlamıyor ([s] basınca hiçbir şey olmuyor)**

```
Dashboard çıktısı: [WARN] QR map not ready. Call setup_qr_map() first!
```

**Neden:** Jüri koordinatları yüklenmemiş

**Çözüm:** Geri dön ve [m] tuşuna basarak koordinatları gir.

---

### **Sorun: 3 drone birlikte kalkmıyor (asenkron kalkar)**

Eğer drone2 ve drone3 drone1'den çok daha geç kalkıyorsa:

**Tanı:** Xterm'de zaman farkını kontrol et
```
[timestamp1] drone1: Taking off
[timestamp3] drone3: Taking off
# Fark > 2 saniye ise problem var
```

**Çözüm:**
```bash
# MAVROS servis ready zamanı artır
# src/my_swarm_pkg/launch/swarm_competition.launch.py
# Satır ~280'de:
#   delay=9.0  # 9 saniye olmalı
#   (11.0 olarak değiştir ve recompile et)
```

---

### **Sorun: "Collision avoidance triggered" uyarısı**

Drone'ler çok yakın hareket ediyor (formation düzgün değil)

**Çözüm:** YAML config'te formation offset'lerini kontrol et
```bash
less ~/gz_ws/src/my_swarm_pkg/config/swarm_params.yaml
# offset_y_drone2, offset_y_drone3 farkları kontrol et
```

---

## ✅ BAŞARI KRİTERLERİ

Sistemin başarılı çalıştığını anlamak için:

| Kriter | Beklenti | Kontrol Yöntemi |
|--------|----------|-----------------|
| **Gazebo başlar** | Pencere açılır, 3 drone görülür | Göz ile doğrula |
| **SITL bağlanır** | 3 port (14550/14560/14570) aktif | `netstat \| grep 14550` |
| **MAVROS bağlanır** | `/drone1/mavros/state` topic var | `ros2 topic list \| grep state` |
| **Koordinat yüklenir** | SetQRMap başarılı | `[SUCCESS]` message xterm'de |
| **TASK1 başlar** | drone1/2/3 IDLE → ARMING → FLYING | Gazebo'da motor sesi + hareket |
| **Senkron kalkış** | 3 drone < 1 sn fark ile kalkar | Xterm timestamp'lerini karşılaştır |
| **Rota takibi** | Drone'ler QR1'e doğru hareket eder | Gazebo'da gözle izle |
| **QR algılama** | Her waypoint'te "QR detected" | Xterm'de log mesajları |

---

## 📱 DASHBOARD KOMUTLARI (Mission FSM xterm'de)

```
[m]  — Map Setup: Jüri koordinatlarını gir (FORMAT: x,y,z)
[s]  — Start TASK1: 3 drone senkron kalkar, QR rota başlar
[p]  — Pause: Drone'leri HOVER moduna al (askıda tuttur)
[r]  — Resume: HOVER'dan rota devam et
[h]  — Home: Tüm drone'ler HOME'a dönsün
[q]  — Quit: Simulasyonu sonlandır

[?]  — Help: Tüm komutları göster
```

---

## 🎯 JÜRİ KOORDİNATLARI ÖRNEK VERI

Eğer jüri bu koordinatları verse:

```
QR1: 20.0, 15.0, 20.0
QR2: 35.0, 30.0, 25.0
QR3: 50.0, 15.0, 20.0
QR4: 65.0, 30.0, 22.0
QR5: 80.0, 15.0, 20.0
QR6: 95.0, 30.0, 25.0
```

**Sistem yapacağı:**

1. Drone1, drone2, drone3 QR1'e doğru uçar (20, 15, 20)
2. QR1'e yaklaştığında (5m): "QR1 detected"
3. Rota QR2'ye geçer (35, 30, 25)
4. Drone'ler QR2'ye doğru uçar (formation oluştur)
5. ... (6. QR'a kadar devam)
6. QR6'dan sonra: HOME (0, 0, 0) konumuna iniş

**Toplam misyon süresi:** ~5-8 dakika (waypoint arası mesafeye bağlı)

---

## 📞 ACIL DURUM KONTAKLARI

- **ROS2 service'i durdurmak:** `Ctrl+C` tuşu
- **Gazebo'yu kapatmak:** Gazebo penceresi ✕ tuşu
- **Simulation durumu:**
  ```bash
  ros2 node list | wc -l  # Tüm node'ler başlatılmış mı?
  ros2 topic list         # Topic'ler yayınlanıyor mu?
  ```

---

## 🏆 BAŞARILAR DİLEYİZ!

**İnşaallah TEKNOFEST 2026'de birincilik kazanırsınız! 🚀🎉**

Bu protokole uymak, sistemin güvenilir çalışmasını sağlar. 
Herhangi bir soru veya sorun için:

1. `build/` dizinini temizle: `rm -rf build/ install/ log/`
2. Yeniden derle: `colcon build --packages-select swarm_msgs my_swarm_pkg`
3. Protokolü baştan takip et

---

**Dokuman Versiyon:** v1.0  
**Son Güncelleme:** Ocak 2026  
**Hazırlayan:** Seyda (İnşaallah başarılı olur! 🙏)
