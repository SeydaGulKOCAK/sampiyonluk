# JoinRequest.msg
# Yayıncı : local_fsm (STANDBY state'indeki yedek drone)
# Okuyan  : intent_coordinator (TÜM İHA'lar)
#
# STANDBY'daki yedek drone periyodik olarak bu mesajı gönderir:
# "Ben hazırım, ne zaman sürüye katılayım?"
# intent_coordinator bir drone DISARM_WAIT'e geçince yedek drone'u
# çağırır ve SwarmIntent.join_drone_id'yi doldurur.

std_msgs/Header header

uint8  drone_id         # Hangi yedek drone talep ediyor?
float32 battery_pct     # Batarya yüzdesi (%) — intent_coordinator
                        # en dolu bataryalı yedek drone'u seçebilir
bool   ready            # Gerçekten hazır mı? (ARM edilebilir durumdaysa True)
