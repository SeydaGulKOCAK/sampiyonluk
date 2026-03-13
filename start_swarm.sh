#!/usr/bin/env bash
# ============================================================
#  start_swarm.sh  —  Sürü İHA Sistemini Başlatma Scripti
#  TEKNOFEST 2026 — my_swarm_pkg
#
#  KULLANIM:
#    bash ~/gz_ws/src/my_swarm_pkg/start_swarm.sh
#    bash ~/gz_ws/src/my_swarm_pkg/start_swarm.sh --no-sitl   # SITL atla
#    bash ~/gz_ws/src/my_swarm_pkg/start_swarm.sh --stop      # Durdur
#
#  Bu script şunları başlatır (sırayla):
#    1. ROS2 ortam + DDS loopback ayarı
#    2. ArduCopter SITL x3 (arka planda, ayrı xterm'lerde)
#    3. MAVROS x3 (arka planda)
#    4. Per-drone node'lar x3 (drone_interface, local_fsm, ...)
#    5. GCS node'ları (swarm_takeoff, waypoint_navigator, mission_fsm)
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$SCRIPT_DIR"
WS="$HOME/gz_ws"
LOG_DIR="$HOME/.swarm_logs"
mkdir -p "$LOG_DIR"

# ── Argüman parse ────────────────────────────────────────────
WITH_SITL=true
WITH_GZ=true
WITH_MAVROS=true
for arg in "$@"; do
  case $arg in
    --no-sitl)   WITH_SITL=false ;;
    --no-gz)     WITH_GZ=false ;;
    --no-mavros) WITH_MAVROS=false ;;
    --stop)
      echo "🛑 Swarm durdurulıyor..."
      pkill -f "sim_vehicle.py" 2>/dev/null || true
      pkill -f "mavros_node"    2>/dev/null || true
      pkill -f "drone_interface\|local_fsm\|intent_coordinator\|formation_controller" 2>/dev/null || true
      pkill -f "qr_perception\|collision_avoidance\|safety_monitor" 2>/dev/null || true
      pkill -f "mission_fsm\|waypoint_navigator\|swarm_takeoff" 2>/dev/null || true
      pkill -f "gz sim"         2>/dev/null || true
      echo "✅ Durduruldu"
      exit 0
      ;;
  esac
done

# ── Renk kodları ─────────────────────────────────────────────
GREEN='\033[92m'; YELLOW='\033[93m'; CYAN='\033[96m'; RESET='\033[0m'; BOLD='\033[1m'

banner() { echo -e "\n${BOLD}${CYAN}══ $1 ══${RESET}"; }
ok()     { echo -e "  ${GREEN}✅ $1${RESET}"; }
info()   { echo -e "  ${YELLOW}▶  $1${RESET}"; }

# ── Ortam yükle ───────────────────────────────────────────────
banner "ORTAM AYARI"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# DDS loopback
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://${PKG_DIR}/config/cyclonedds_localhost.xml"

# Gazebo model yolları
export GZ_SIM_RESOURCE_PATH=\
"$WS/install/ardupilot_gazebo/share/ardupilot_gazebo/models:"\
"$WS/install/ardupilot_gazebo/share/ardupilot_gazebo/worlds:"\
"$HOME/ardupilot_gazebo/models:"\
"$HOME/ardupilot_gazebo/worlds"

export PATH="$PATH:$HOME/ardupilot/Tools/autotest"
export SWARM_SIZE=3

ok "ROS_LOCALHOST_ONLY=1"
ok "CYCLONEDDS loopback XML yüklendi"
ok "GZ_SIM_RESOURCE_PATH ayarlandı"

# ── 1. Gazebo ────────────────────────────────────────────────
if [[ "$WITH_GZ" == "true" ]]; then
  banner "GAZEBO"
  if pgrep -x "gz" > /dev/null 2>&1; then
    info "Gazebo zaten çalışıyor, atlanıyor"
  else
    WORLD="$WS/src/my_swarm_pkg/worlds/world_base.sdf"
    gz sim -r "$WORLD" > "$LOG_DIR/gz.log" 2>&1 &
    GZ_PID=$!
    info "Gazebo başlatıldı (PID=$GZ_PID)"
    sleep 4
    ok "Gazebo hazır"
  fi
fi

# ── 2. ArduCopter SITL ───────────────────────────────────────
if [[ "$WITH_SITL" == "true" ]]; then
  banner "ARDUPILOT SITL (3 drone)"
  for i in 0 1 2; do
    SYSID=$((i+1))
    PORT=$((14550 + i*10))
    info "SITL drone${SYSID} (I${i}, port ${PORT}) başlatılıyor..."
    bash -c "cd ~/ardupilot && \
      sim_vehicle.py -v ArduCopter \
        --model json \
        -I${i} --sysid ${SYSID} \
        --out=udp:127.0.0.1:${PORT} \
        --no-mavproxy \
        -w \
      > $LOG_DIR/sitl_drone${SYSID}.log 2>&1" &
    sleep 2
  done
  ok "3x SITL başlatıldı (loglar: $LOG_DIR/sitl_*.log)"
  info "SITL hazır olması için 15s bekleniyor..."
  sleep 15
  ok "SITL hazır"
fi

# ── 3. MAVROS ────────────────────────────────────────────────
if [[ "$WITH_MAVROS" == "true" ]]; then
  banner "MAVROS (3 drone)"
  for i in 1 2 3; do
    PORT=$((14550 + (i-1)*10))
    NS="drone${i}"
    info "MAVROS $NS (sysid=$i, port=$PORT)"
    ros2 run mavros mavros_node \
      --ros-args \
      -r __ns:=/${NS} \
      -p fcu_url:="udp://127.0.0.1:${PORT}@" \
      -p tgt_system:=${i} \
      > "$LOG_DIR/mavros_drone${i}.log" 2>&1 &
    sleep 1
  done
  ok "MAVROS başlatıldı"
  sleep 3
fi

# ── 4. Per-drone node'ları ───────────────────────────────────
banner "PER-DRONE NODE'LAR"

start_drone_nodes() {
  local NS=$1 ID=$2 HX=$3 HY=$4
  local ENV_ARGS="DRONE_NS=$NS DRONE_ID=$ID SWARM_SIZE=3 HOME_X=$HX HOME_Y=$HY HOME_Z=0.0"
  ENV_ARGS+=" FENCE_X_MIN=-5 FENCE_X_MAX=125 FENCE_Y_MIN=-5 FENCE_Y_MAX=95 FENCE_Z_MIN=0.5 FENCE_Z_MAX=60"

  info "$NS → drone_interface"
  env $ENV_ARGS ros2 run my_swarm_pkg drone_interface \
    --ros-args -r __ns:=/${NS} \
    > "$LOG_DIR/${NS}_drone_interface.log" 2>&1 &
  sleep 0.3

  info "$NS → local_fsm"
  env $ENV_ARGS ros2 run my_swarm_pkg local_fsm \
    --ros-args -r __ns:=/${NS} \
    > "$LOG_DIR/${NS}_local_fsm.log" 2>&1 &
  sleep 0.3

  info "$NS → intent_coordinator"
  env $ENV_ARGS ros2 run my_swarm_pkg intent_coordinator \
    --ros-args -r __ns:=/${NS} \
    > "$LOG_DIR/${NS}_intent_coordinator.log" 2>&1 &
  sleep 0.3

  info "$NS → formation_controller"
  env $ENV_ARGS ros2 run my_swarm_pkg formation_controller \
    --ros-args -r __ns:=/${NS} \
    -p drone_id:=${ID} -p swarm_size:=3 \
    -p formation_type:=OKBASI -p drone_spacing:=6.0 \
    > "$LOG_DIR/${NS}_formation_controller.log" 2>&1 &
  sleep 0.3

  info "$NS → qr_perception"
  env $ENV_ARGS ros2 run my_swarm_pkg qr_perception \
    --ros-args -r __ns:=/${NS} \
    -p drone_id:=${ID} \
    > "$LOG_DIR/${NS}_qr_perception.log" 2>&1 &
  sleep 0.3

  info "$NS → collision_avoidance"
  env $ENV_ARGS ros2 run my_swarm_pkg collision_avoidance \
    --ros-args -r __ns:=/${NS} \
    -p drone_id:=${ID} -p swarm_size:=3 \
    > "$LOG_DIR/${NS}_collision_avoidance.log" 2>&1 &
  sleep 0.3

  info "$NS → safety_monitor"
  env $ENV_ARGS ros2 run my_swarm_pkg safety_monitor \
    --ros-args -r __ns:=/${NS} \
    -p drone_id:=${ID} \
    > "$LOG_DIR/${NS}_safety_monitor.log" 2>&1 &
  sleep 0.2

  ok "$NS — 7 node başlatıldı"
}

start_drone_nodes drone1 1 0.0 0.0
sleep 0.5
start_drone_nodes drone2 2 4.0 0.0
sleep 0.5
start_drone_nodes drone3 3 8.0 0.0

# ── 5. GCS node'ları ─────────────────────────────────────────
banner "GCS NODE'LARI"
GCS_ENV="SWARM_SIZE=3 FENCE_X_MIN=-5 FENCE_X_MAX=125 FENCE_Y_MIN=-5 FENCE_Y_MAX=95"

info "swarm_takeoff"
env $GCS_ENV ros2 run my_swarm_pkg swarm_takeoff \
  --ros-args -p swarm_size:=3 -p takeoff_alt_m:=10.0 \
  > "$LOG_DIR/gcs_swarm_takeoff.log" 2>&1 &
sleep 0.5

info "waypoint_navigator"
env $GCS_ENV ros2 run my_swarm_pkg waypoint_navigator \
  --ros-args -p swarm_size:=3 \
  > "$LOG_DIR/gcs_waypoint_navigator.log" 2>&1 &
sleep 0.5

info "mission_fsm (interaktif — bu terminal)"
env $GCS_ENV ros2 run my_swarm_pkg mission_fsm \
  --ros-args -p swarm_size:=3

# mission_fsm interaktif olduğu için foreground'da çalışır
# script buradan devam etmez (mission_fsm kapatılınca biter)

banner "TÜM SİSTEM BAŞLATILDI"
echo ""
echo "  Log dosyaları: $LOG_DIR/"
echo "  Durdurmak için: bash $0 --stop"
