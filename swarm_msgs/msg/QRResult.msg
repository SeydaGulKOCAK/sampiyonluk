# QRResult.msg
# Yayıncı : qr_perception (Kamera olan İHA)
# Okuyan  : intent_coordinator (TÜM İHA'lar)
#
# QR kodu okunduğunda parse edilmiş içeriği taşır.
# intent_coordinator sadece kendi takım ID'sine ait mesajları işler.
#
# FAILOVER:
#   Kamera drone'u SAFETY_HOLD veya PILOT_OVERRIDE'a geçince
#   yedek drone'un qr_perception'ı devreye girer — aynı topic'i kullanır.
#
# QR FORMATI (Şartname):
#   team_id | formasyon | irtifa | manevra | birey_çıkar | sonraki_qr_no

std_msgs/Header header

string  team_id             # Takım ID'si — kendi ID'mizle eşleşmeli, yoksa DROP

uint32  qr_id               # Bu QR'ın numarası (1, 2, 3...)
                            # intent_coordinator: qr_id <= son_işlenen → DROP

# ── FORMASYON KOMUTU ──────────────────────────────────────────────────────
bool    formation_active    # Bu QR'da formasyon değişiyor mu?
string  formation_type      # 'OKBASI' / 'V' / 'CIZGI'
float32 drone_spacing       # Drone'lar arası mesafe (metre)

# ── İRTİFA KOMUTU ─────────────────────────────────────────────────────────
bool    altitude_active     # Bu QR'da irtifa değişiyor mu?
float32 altitude            # Yeni hedef irtifa (metre)

# ── MANEVRA KOMUTU ────────────────────────────────────────────────────────
# Pitch/Roll manevrası: formasyon yatay düzlemde eğilir
# formation_controller 3D rotasyon matrisiyle hesaplar
bool    maneuver_active
float32 pitch_deg           # Eğim açısı (derece) — negatif = öne eğim
float32 roll_deg            # Yatış açısı (derece) — pozitif = sağa

# ── BİREY ÇIKARMA KOMUTU ─────────────────────────────────────────────────
bool    detach_active       # Bu QR'da birey ayrılıyor mu?
uint8   detach_drone_id     # Ayrılacak drone'un ID'si
string  zone_color          # Hangi renkli alana inecek: 'RED' / 'BLUE'

# ── SONRAKI QR ────────────────────────────────────────────────────────────
uint32  next_qr_id          # Sonraki QR'ın numarası
                            # 0 = Bu SON QR → görev tamamlandı → RETURN_HOME

geometry_msgs/Point qr_position       # Bu QR'ın ENU koordinatı (okunduğu nokta)

geometry_msgs/Point next_qr_position  # Bir sonraki QR'ın ENU koordinatı
                                       # intent_coordinator bunu target_pos olarak kullanır
                                       # next_qr_id = 0 ise sıfır-nokta (RETURN_HOME)

float32 wait_seconds                  # QR görevini icra ettikten sonra bekle [s]
