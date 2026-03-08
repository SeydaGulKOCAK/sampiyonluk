#!/bin/bash
#╔═══════════════════════════════════════════════════════════════════════════╗
#║                  start_swarm_competition.sh                              ║
#║          Yarışma Gülü — 3 Drone Swarm Startup Script                    ║
#║                    İnşaallah sorunsuz çalışır! 🙏                        ║
#╚═══════════════════════════════════════════════════════════════════════════╝

set -e  # Hata varsa dur

# Renk kodları (terminal output'u güzel yap)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# ============================================================================
# KONTROL FONKSİYONLARI
# ============================================================================

info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

success() {
    echo -e "${GREEN}✅${NC} $1"
}

warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

error() {
    echo -e "${RED}❌${NC} $1"
    exit 1
}

# ============================================================================
# BAŞLANGICI KONTROLÜ
# ============================================================================

info "TEKNOFEST 2026 Swarm Simulasyonu Başlatılıyor..."
echo ""

# ROS2 çevre değişkenlerini kontrol et
if [ -z "$ROS_LOCALHOST_ONLY" ]; then
    warning "ROS2 çevre değişkenleri ayarlanmamış"
    info "Şimdi çalışılan dosya: $(pwd)"
    
    if [ -f "install/setup.bash" ]; then
        info "install/setup.bash bulundu, yükleniyor..."
        source install/setup.bash
    elif [ -f "~/gz_ws/install/setup.bash" ]; then
        info "~/gz_ws/install/setup.bash bulundu, yükleniyor..."
        source ~/gz_ws/install/setup.bash
    else
        error "ROS2 ortamı kurulamadı. Şunları deneyin:"
        echo "  1. cd ~/gz_ws"
        echo "  2. . install/setup.bash"
        echo "  3. bash $0"
    fi
fi

success "ROS2 ortamı: $ROS_DISTRO ($ROS_LOCALHOST_ONLY)"

# ============================================================================
# GERÇEKTİME KONTROL HATLARI
# ============================================================================

echo ""
info "KONTROL KONTROL LISTESI:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Gazebo var mı?
if command -v gz &> /dev/null; then
    success "Gazebo bulundu ($(gz --version))"
else
    error "Gazebo yüklenmemiş! Şunları çalıştırın:"
    echo "  sudo apt install gazebo"
fi

# MAVROS var mı?
if ros2 pkg list | grep -q mavros; then
    success "MAVROS kurulu"
else
    error "MAVROS yüklenmemiş! Şunları çalıştırın:"
    echo "  sudo apt install ros-humble-mavros ros-humble-mavros-extras"
fi

# ArduPilot sim_vehicle.py var mı?
if command -v sim_vehicle.py &> /dev/null; then
    success "ArduPilot SITL bulundu"
else
    warning "ArduPilot SITL'in PATH'de olması bekleniyor"
    info "Devam ediyoruz, runtime'da sorunu görürüz..."
fi

# xterm var mı?
if command -v xterm &> /dev/null; then
    success "xterm yüklü (dashboard için)"
else
    warning "xterm yüklenmemiş. Dashboard terminal'e açılacak (GUI yok)"
    warning "Yüklemek için: sudo apt install xterm"
fi

# Paketler derlenmiş mi?
if [ -d "build/my_swarm_pkg" ] && [ -d "install/my_swarm_pkg" ]; then
    success "my_swarm_pkg derlenmiş"
else
    warning "my_swarm_pkg derlenmemiş veya yeniden derleme gerekli"
    info "Derleniyor: colcon build --packages-select swarm_msgs my_swarm_pkg"
    colcon build --packages-select swarm_msgs my_swarm_pkg
    . install/setup.bash
    success "Derleme tamamlandı"
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# ============================================================================
# YANIŞLAMA
# ============================================================================

echo ""
echo -e "${BOLD}╔═══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║${NC}     SWARM LAUNCH BAŞLATILIYOR (T-30 saniye)"
echo -e "${BOLD}║${NC}"
echo -e "${BOLD}║${NC}  🎯 Yapılacaklar:"
echo -e "${BOLD}║${NC}    1. Gazebo (world_base.sdf) açılır"
echo -e "${BOLD}║${NC}    2. SITL drone1, drone2, drone3 başlar"
echo -e "${BOLD}║${NC}    3. MAVROS her drone için bağlanır"
echo -e "${BOLD}║${NC}    4. GCS nodes başlar (qr_perception, etc)"
echo -e "${BOLD}║${NC}    5. Mission FSM xterm penceresi açılır"
echo -e "${BOLD}║${NC}"
echo -e "${BOLD}║${NC}  ⏱️  BEKLEME: ~15-20 saniye"
echo -e "${BOLD}║${NC}"
echo -e "${BOLD}║${NC}  📝 Sonra dashboard'da:"
echo -e "${BOLD}║${NC}    [m] → Jüri koordinatlarını gir"
echo -e "${BOLD}║${NC}    [s] → TASK1 başlat (3 drone senkron kalkar)"
echo -e "${BOLD}║${NC}"
echo -e "${BOLD}╚═══════════════════════════════════════════════════════════╝${NC}"
echo ""

read -p "Başlamak için ENTER'a basın (q + ENTER = iptal): " -t 10 confirm
if [ "$confirm" = "q" ] || [ "$confirm" = "Q" ]; then
    warning "İptal edildi."
    exit 0
fi

echo ""
info "🚀 Sistem başlatılıyor..."
echo ""

# ============================================================================
# LAUNCH BAŞLAT
# ============================================================================

# Log dosyası
LOG_DIR="$HOME/swarm_logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/swarm_$(date +%Y%m%d_%H%M%S).log"

info "Log dosyası: $LOG_FILE"

# Launch komutunu çalıştır
ros2 launch my_swarm_pkg swarm_competition.launch.py 2>&1 | tee "$LOG_FILE"

# Eğer buraya ulaştıysak, launch kapandı (Ctrl+C veya hata)
echo ""
warning "Launch sona erdi."
echo ""
echo -e "${YELLOW}Tüm node'ler kapatılıyor...${NC}"

# Temizlik (opsiyonel)
sleep 2

success "Simulasyon tamamlandı."
echo ""
info "Log dosyası kayıtlı: $LOG_FILE"
info "Sorun çıkarmış ise, kontrol et: cat $LOG_FILE | grep ERROR"
