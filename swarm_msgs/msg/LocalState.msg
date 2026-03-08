# LocalState.msg
# Yayıncı : local_fsm
# Okuyan  : intent_coordinator (TÜM İHA'lar), swarm_comm (GCS izleme)
#
# Her drone'un şu an hangi görev adımında olduğunu bildirir.
# intent_coordinator bu mesajları heartbeat olarak kullanır:
# 500ms boyunca mesaj gelmezse drone düşmüş sayılır.
#
# STATE DEĞERLERİ:
#   STANDBY      → Yerde bekliyor, henüz görev almadı (yedek drone)
#   IDLE         → ARM edildi, kalkışa hazır
#   FLYING       → Havada, sürüyle birlikte görev yapıyor
#   DETACH       → Sürüden ayrıldı, iniş bölgesine gidiyor
#   LAND_ZONE    → İniş bölgesi üzerinde, precision_landing aktif
#   DISARM_WAIT  → İnis yapıldı, DISARM bekleniyor
#   REARM        → Yeniden ARM ediliyor, sürüye katılacak
#   REJOIN       → Sürüye katılma manevrası yapıyor
#   RETURN_HOME  → Tüm görevler bitti, eve dönüyor
#   LANDING      → Eve geldi, iniş yapıyor
#   SAFETY_HOLD  → Güvenlik olayı: batarya kritik / geofence ihlali
#   PILOT_OVERRIDE → RC kill-switch / GUIDED dışı mod tespit edildi

std_msgs/Header header   # timestamp + frame_id

uint8 drone_id           # Hangi drone? (1, 2, 3...)
string state             # Yukarıdaki state değerlerinden biri

uint32 seq               # Monotonic artan sıra numarası
                         # intent_coordinator çift filtre için kullanır:
                         # seq < son_seq → eski mesaj, DROP et
