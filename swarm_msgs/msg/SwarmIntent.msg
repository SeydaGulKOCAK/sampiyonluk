# SwarmIntent.msg
# Yayıncı : intent_coordinator (SADECE LİDER İHA)
# Okuyan  : local_fsm (TÜM İHA'lar), formation_controller (TÜM)
#
# Tüm sürünün ne yapacağını söyleyen ANA KOMUT paketidir.
# Bu mesajı SADECE lider İHA'daki intent_coordinator yazar.
# GCS ASLA bu topic'e yazmaz → §5.3 dağıtık mimari kuralı.
#
# ÇİFT FİLTRE KURALI (local_fsm'de uygulanır):
#   1) seq < son_seq → ESKİ LİDERDEN KALAN MESAJ → DROP et
#   2) seq == son_seq ve header.stamp < son_timestamp → DROP et

std_msgs/Header header          # timestamp (çift filtre için kullanılır)

uint32 seq                      # Monotonic artan sıra no (lider değişiminde seq+1'den devam)
uint8  leader_id                # Bu mesajı kim üretiyor? (lider drone'un ID'si)

# ── GÖREV BİLGİSİ ──────────────────────────────────────────────
string task_id                  # Aktif görev: 'QR_NAVIGATE' / 'DETACH' / 'REJOIN' /
                                # 'RETURN_HOME' / 'IDLE' / 'MANEUVER'

# ── FORMASYON ──────────────────────────────────────────────────
string formation_type           # 'OKBASI' / 'V' / 'CIZGI'
float32 drone_spacing           # Drone'lar arası mesafe (metre) — QR'dan gelir
float32 target_yaw              # Sürünün ilerleyeceği yön (radyan, ENU'da kuzeyden)
                                # formation_controller offset rotasyonu için kullanır

# ── HEDEF POZİSYON ─────────────────────────────────────────────
geometry_msgs/Point target_pos  # Sonraki QR / hedef koordinat (ENU, metre)
float32 drone_altitude          # Hedef irtifa (metre) — QR'dan gelir

# ── BİREY ÇIKARMA / EKLEME ──────────────────────────────────────
uint8   detach_drone_id         # Ayrılacak drone'un ID'si (0 = kimse ayrılmıyor)
string  zone_color              # İniş yapılacak zone rengi: 'RED' / 'BLUE' / ''

# ── MANEVRA ────────────────────────────────────────────────────
bool    maneuver_active         # Pitch/Roll manevrası aktif mi?
float32 maneuver_pitch_deg      # Pitch açısı (derece) — negatif = öne eğim
float32 maneuver_roll_deg       # Roll açısı (derece) — sağa pozitif

# ── SÜRÜ DURUMU ────────────────────────────────────────────────
uint8   active_drone_count      # Şu an FLYING olan drone sayısı
                                # formation_controller fallback için kullanır:
                                # < 3 → 2'li LINE formasyonuna geç

uint8   join_drone_id           # Sürüye katılacak yedek drone'un ID'si (0 = yok)

# ── QR BİLGİSİ ─────────────────────────────────────────────────
uint32  qr_seq                  # QR sıra numarası (her QR için artar)
                                # Aynı QR'ı iki kez işlemeyi engeller

# ── BEKLEME ────────────────────────────────────────────────────
builtin_interfaces/Time wait_until  # Bu zamana kadar mevcut konumda bekle
                                    # Manevra / QR okuma sırasında kullanılır
