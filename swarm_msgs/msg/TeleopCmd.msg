# TeleopCmd.msg
# Yayıncı : swarm_teleop (GCS) — SADECE TASK2_ACTIVE modunda
# Okuyan  : formation_controller (TÜM RPi4'ler)
#
# Görev 2 (Yarı Otonom Sürü Kontrolü) için joystick/RC kumanda komutunu taşır.
# Ham hız vektörü içerir — world-frame setpoint DEĞİL.
#
# ÖNEMLİ NOTLAR:
#   execute_at: Wi-Fi gecikmesi kompansasyonu için.
#     Her drone bu timestamp'te komutu uygular → formasyon bütünlüğü korunur.
#   TASK1_ACTIVE'de swarm_teleop bu topic'e YAZMAZ (mode gating ile kapalı).
#
# MOD AÇIKLAMASI:
#   MOVE     : Joystick tüm sürüyü hareket ettirir.
#              Formasyon bozulmaz, sürünün merkezi kayar.
#              vx/vy/vz/yaw_rate ile kontrol edilir.
#   MANEUVER : Formasyon şekli değişir.
#              Sürünün merkezi sabit kalır.
#              new_formation_type ve/veya new_spacing ile şekil değiştirilir.

std_msgs/Header header

# ── MOD ────────────────────────────────────────────────────────
string  mode            # 'MOVE' veya 'MANEUVER'

# ── HAREKET KOMUTU (MOVE modunda kullanılır) ───────────────────
# Koordinat: formation-local çerçeve (ileri/sol/yukarı)
# Birim: m/s (normalize edilmiş joystick girdisi × max_speed)
float32 vx              # ileri (+) / geri (-)     hızı [m/s]
float32 vy              # sol   (+) / sağ  (-)     hızı [m/s]
float32 vz              # yukarı(+) / aşağı(-)     hızı [m/s]
float32 yaw_rate        # CCW(+)  / CW(-)  dönüş hızı [rad/s]
                        # → formasyon merkezi etrafında tüm sürüyü döndürür

# ── ZAMANLAMA (Wi-Fi gecikmesi kompansasyonu) ──────────────────
builtin_interfaces/Time execute_at
                        # Bu timestamp'te komut uygulanır.
                        # 0 → hemen uygula (gecikme yok)

# ── FORMASYON DEĞİŞİKLİĞİ (MANEUVER modunda kullanılır) ───────
string  new_formation_type  # 'OKBASI' / 'V' / 'CIZGI' — boş ise değişmez
float32 new_spacing         # 0.0 ise spacing değişmez
