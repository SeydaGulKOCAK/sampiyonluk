# ColorZoneList.msg
# Yayıncı : qr_perception   → /perception/color_zones  (1 Hz)
# Okuyan  : intent_coordinator (DETACH görevinde hedef zone seçimi için)
#
# Simülasyonda tüm renk bölgeleri YAML'dan yüklenip sürekli yayınlanır.
# Gerçek sistemde kamera ile tespit edilir.

std_msgs/Header header
ColorZone[]     zones           # Tespit edilen tüm renk bölgeleri
