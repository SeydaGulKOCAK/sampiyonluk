# ColorZone.msg
# Yayıncı : qr_perception
# Okuyan  : intent_coordinator (DETACH görevinde iniş bölgesi seçimi için)
#
# Rota boyunca tespit edilen tek bir renk bölgesi.
# Sürü bu bölgeleri hafızaya alır; birey çıkarma komutunda kullanır.
#
# Şartname §5.1.2:
#   "...birey ayrılacak iniş alanının rengini ve konumunu kaydeder"

geometry_msgs/Point position    # ENU koordinat [m] — z genellikle 0 (zemin)
string  color                   # 'RED' veya 'BLUE'
float32 radius                  # Hassas iniş bölgesi yarıçapı [m]
