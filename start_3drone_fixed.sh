#!/bin/bash
# ╔══════════════════════════════════════════════════════════════╗
# ║  3 DRONE BAŞLATMA — MAVROS FIX EDİLMİŞ VERSİYON            ║
# ║  Gazebo → SITL(x3) → MAVROS(x3)                             ║
# ╚══════════════════════════════════════════════════════════════╝

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; BOLD='\033[1m'; NC='\033[0m'

ok()   { echo -e "${GREEN}✅${NC} $1"; }
info() { echo -e "${BLUE}ℹ${NC}  $1"; }
warn() { echo -e "${YELLOW}⚠${NC}  $1"; }
err()  { echo -e "${RED}❌${NC} $1"; }

cleanup() {
    echo ""
    warn "Kapatılıyor..."
    pkill -f "sim_vehicle" 2>/dev/null
    pkill -f "arducopter"  2>/dev/null
    pkill -f "mavproxy"    2>/dev/null
    pkill -f "mavros_node" 2>/dev/null
    pkill -f "gz sim"      2>/dev/null
    sleep 1
    ok "Temizlendi."
}
trap cleanup EXIT INT TERM

# ── Ortam ──────────────────────────────────────────────────
set +u  # ROS setup scriptleri unbound variable hatası verir
source /opt/ros/humble/setup.bash
source ~/gz_ws/install/setup.bash 2>/dev/null || true

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PATH="$PATH:$HOME/ardupilot/Tools/autotest"

DIR="/home/seyda/sampiyonluk"

export GZ_SIM_RESOURCE_PATH="${DIR}/models:$HOME/gz_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/models:$HOME/gz_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/worlds:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/gz_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo:$HOME/ardupilot_gazebo/build:$HOME/gz_ws/build/ardupilot_gazebo"

echo ""
echo -e "${BOLD}╔═══════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║  🚁  3 DRONE SİMÜLASYON — BİSMİLLAH! 🙏            ║${NC}"
echo -e "${BOLD}╚═══════════════════════════════════════════════════════╝${NC}"

# ═══════════════════════════════════════════════════════════
# 1) GAZEBO
# ═══════════════════════════════════════════════════════════
echo ""
echo -e "${BOLD}━━━ [1/3] GAZEBO ━━━${NC}"

cd "$DIR"
gz sim -r --render-engine ogre worlds/world_base.sdf > /tmp/gz_sim.log 2>&1 &
GZ_PID=$!

for i in $(seq 1 20); do
    if ! kill -0 $GZ_PID 2>/dev/null; then
        err "Gazebo çöktü! Log: /tmp/gz_sim.log"
        tail -5 /tmp/gz_sim.log
        exit 1
    fi
    printf "\r  ⏳ Gazebo yükleniyor... %2d/20s" "$i"
    sleep 1
done
echo ""
ok "Gazebo çalışıyor (PID: $GZ_PID)"

# ═══════════════════════════════════════════════════════════
# 2) ARDUPILOT SITL (3 drone)
# ═══════════════════════════════════════════════════════════
echo ""
echo -e "${BOLD}━━━ [2/3] SITL (x3) ━━━${NC}"

cd ~/ardupilot

for I in 0 1 2; do
    SYSID=$((I + 1))
    info "Drone${SYSID}: -I${I}, sysid=${SYSID}"
    sim_vehicle.py -v ArduCopter --model json \
        -I${I} --sysid ${SYSID} --speedup 1 \
        -l -35.363262,149.165237,584,0 \
        --no-rebuild --no-mavproxy \
        > /tmp/sitl_drone${SYSID}.log 2>&1 &
    sleep 3
done

info "SITL TCP portları bekleniyor..."
for attempt in $(seq 1 30); do
    OPEN=0
    for port in 5760 5770 5780; do
        ss -tlnp 2>/dev/null | grep -q ":${port}" && OPEN=$((OPEN + 1))
    done
    printf "\r  ⏳ Portlar: %d/3 açık (%2ds)" "$OPEN" "$attempt"
    if [ "$OPEN" -eq 3 ]; then
        break
    fi
    sleep 1
done
echo ""

for port in 5760 5770 5780; do
    if ss -tlnp 2>/dev/null | grep -q ":${port}"; then
        ok "TCP :${port} AÇIK"
    else
        err "TCP :${port} KAPALI! Log: /tmp/sitl_drone*.log"
    fi
done

# ═══════════════════════════════════════════════════════════
# 3) MAVROS (3 drone — LAUNCH dosyasıyla)
# ═══════════════════════════════════════════════════════════
echo ""
echo -e "${BOLD}━━━ [3/3] MAVROS (x3) ━━━${NC}"
info "Plugin whitelist ile başlatılıyor (crash fix)"

PLUGIN_YAML="${DIR}/config/mavros_plugins.yaml"

for I in 1 2 3; do
    PORT=$((5760 + (I - 1) * 10))
    info "drone${I} MAVROS → tcp://127.0.0.1:${PORT}"

    ros2 run mavros mavros_node --ros-args \
        -r __ns:=/drone${I} \
        -r __node:=mavros \
        -p fcu_url:="tcp://127.0.0.1:${PORT}" \
        -p tgt_system:=${I} \
        -p tgt_component:=1 \
        --params-file "${PLUGIN_YAML}" \
        > /tmp/mavros_drone${I}.log 2>&1 &

    # Her MAVROS arasında bekle — aynı anda başlarsa çöker
    sleep 8
    
    # Hâlâ yaşıyor mu kontrol et
    if ps aux | grep "mavros_node" | grep "drone${I}" | grep -v grep > /dev/null 2>&1; then
        ok "drone${I} MAVROS çalışıyor ✓"
    else
        err "drone${I} MAVROS çöktü!"
        warn "Alternatif yöntem deneniyor (plugin_denylist)..."
        
        # Alternatif: denylist ile dene
        ros2 run mavros mavros_node --ros-args \
            -r __ns:=/drone${I} \
            -r __node:=mavros \
            -p fcu_url:="tcp://127.0.0.1:${PORT}" \
            -p tgt_system:=${I} \
            -p tgt_component:=1 \
            -p plugin_denylist:="['companion_process_status','debug_value','obstacle_distance','play_tune','log_transfer','onboard_computer_status','wheel_odometry','vibration']" \
            > /tmp/mavros_drone${I}.log 2>&1 &
        sleep 8
        
        if ps aux | grep "mavros_node" | grep "drone${I}" | grep -v grep > /dev/null 2>&1; then
            ok "drone${I} MAVROS çalışıyor (denylist ile) ✓"
        else
            err "drone${I} MAVROS başarısız. Log:"
            tail -5 /tmp/mavros_drone${I}.log
        fi
    fi
done

# ═══════════════════════════════════════════════════════════
# BAĞLANTI KONTROLÜ
# ═══════════════════════════════════════════════════════════
echo ""
echo -e "${BOLD}━━━ BAĞLANTI KONTROLÜ ━━━${NC}"

sleep 5

for I in 1 2 3; do
    RESULT=$(timeout 8 ros2 topic echo /drone${I}/mavros/state --once 2>/dev/null | grep "connected:" || echo "connected: unknown")
    if echo "$RESULT" | grep -q "true"; then
        ok "drone${I} → FCU BAĞLI 🎉"
    else
        warn "drone${I} → Bağlantı durumu: $RESULT"
    fi
done

# ═══════════════════════════════════════════════════════════
# HAZIR
# ═══════════════════════════════════════════════════════════
echo ""
echo -e "${BOLD}╔═══════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║  SİSTEM HAZIR!                                        ║${NC}"
echo -e "${BOLD}║                                                       ║${NC}"
echo -e "${BOLD}║  Yeni terminal aç:                                    ║${NC}"
echo -e "${BOLD}║    source /opt/ros/humble/setup.bash                  ║${NC}"
echo -e "${BOLD}║    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp       ║${NC}"
echo -e "${BOLD}║    python3 ~/sampiyonluk/test_3drone_takeoff.py       ║${NC}"
echo -e "${BOLD}║                                                       ║${NC}"
echo -e "${BOLD}║  Ctrl+C → Her şeyi kapatır                           ║${NC}"
echo -e "${BOLD}╚═══════════════════════════════════════════════════════╝${NC}"
echo ""

wait
