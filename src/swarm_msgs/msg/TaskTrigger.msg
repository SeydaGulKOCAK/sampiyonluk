# TaskTrigger.msg
# Yayıncı : mission_fsm (GCS — Yer Kontrol İstasyonu)
# Okuyan  : intent_coordinator (TÜM İHA'lar)
#
# Görevi başlatan veya durduran TEK GCS komutu.
# Şartname: TASK1_ACTIVE modunda GCS başka hiçbir kontrol komutu göndermez.
# Bu mesaj gelince intent_coordinator mode_gating'i devreye alır.
#
# YARIŞMA PROSEDÜRÜ:
#   Hakem "başlat" dediğinde operatör bu komutu gönderir.
#   Sonrasında GCS müdahalesi imkânsız hale gelir (mode gating).
#
# TASK1: Otonom sürü görevi (QR navigasyon + birey çıkarma/ekleme)
# TASK2: Yarı otonom görev (joystick ile sürü kontrolü)

std_msgs/Header header

string  task_type   # 'TASK1' veya 'TASK2'
bool    start       # True = görevi başlat, False = görevi durdur
string  team_id     # Takım ID'si (doğrulama için)
