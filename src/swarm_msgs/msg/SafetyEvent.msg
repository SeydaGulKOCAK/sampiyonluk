# SafetyEvent.msg
# Yayıncı : safety_monitor (Her RPi4)
# Okuyan  : local_fsm (kendi RPi4), mission_fsm (GCS)
#
# Kritik güvenlik durumu oluştuğunda yayınlanır.
# local_fsm bu mesajı alınca GCS'ye SORMADAN SAFETY_HOLD'a geçer.
# §5.5.4: Her İHA bağımsız failsafe kararı alabilmelidir.
#
# EVENT_TYPE DEĞERLERİ:
#   BATTERY_CRITICAL  → Batarya eşik altına düştü (örn: %15)
#   GEOFENCE_BREACH   → Drone izin verilen uçuş alanının dışına çıktı
#   OSCILLATION       → Osilasyon tespit edildi (PID yeter siz → tehlikeli)
#   GNSS_DEGRADED     → GPS sinyal kalitesi kritik seviyenin altına düştü
#   ALTITUDE_EXCEEDED → Yazılımsal tavan aşıldı

std_msgs/Header header

uint8  drone_id         # Hangi drone'da olay oluştu?
string event_type       # Yukarıdaki event type değerlerinden biri
string description      # İnsan okunabilir açıklama (log için)
float32 severity        # 0.0-1.0 arası şiddet (1.0 = maksimum tehlike)
