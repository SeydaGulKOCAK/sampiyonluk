# FormationCmd.msg
# Yayıncı : intent_coordinator (LİDER İHA)
# Okuyan  : formation_controller (TÜM İHA'lar)
#
# Aktif formasyon bilgisini taşır.
# SwarmIntent içinde de aynı bilgi var, ama formation_controller
# için ayrı ve sade bir topic daha temiz bir arayüz sağlar.
#
# GÖREV 2'DE: swarm_teleop da bu topic'e yazar (TASK1_ACTIVE'de kapalı).

std_msgs/Header header

string  formation_type      # 'OKBASI' / 'V' / 'CIZGI'
float32 drone_spacing       # Drone'lar arası mesafe (metre)
float32 target_yaw          # Formasyonun baktığı yön (radyan, ENU)
                            # formation_controller offset vektörlerini bu açıyla döndürür
uint8   active_drone_count  # Şu an havada kaç drone var?
                            # < 3 ise formation_controller 2'li LINE'a geçer (fallback)
