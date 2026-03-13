#!/usr/bin/env bash
# ============================================================
#  swarm_env.sh  —  Sürü sistemi ortam değişkenleri
#  Her terminalde:  source ~/gz_ws/src/my_swarm_pkg/config/swarm_env.sh
# ============================================================

# ── ROS2 workspace ──────────────────────────────────────────
source /opt/ros/humble/setup.bash
# source ~/gz_ws/install/setup.bash  # Eğer varsa uncomment et

# ── DDS: loopback-only (Wi-Fi kapalıyken multicast hatası önleme) ──
export ROS_LOCALHOST_ONLY=1
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # CycloneDDS kurulu değil
# export CYCLONEDDS_URI="file://$HOME/sampiyonluk/config/cyclonedds_localhost.xml"

# ── Gazebo model yolları ─────────────────────────────────────
export GZ_SIM_RESOURCE_PATH=\
"$HOME/sampiyonluk/models:"\
"$HOME/ardupilot_gazebo/models:"\
"$HOME/ardupilot_gazebo/worlds"

# ── Gazebo plugin yolları (ArduPilotPlugin için) ─────────────
export GZ_SIM_SYSTEM_PLUGIN_PATH=\
"$HOME/ardupilot_gazebo/build:"\
"$GZ_SIM_SYSTEM_PLUGIN_PATH"

# ── ArduPilot SITL ───────────────────────────────────────────
export PATH="$PATH:$HOME/ardupilot/Tools/autotest"

# ── Swarm parametreleri ──────────────────────────────────────
export SWARM_SIZE=3

echo "✅ Swarm ortamı yüklendi (ROS_LOCALHOST_ONLY=1)"
echo "   DDS: CycloneDDS loopback"
echo "   GZ_SIM_RESOURCE_PATH ayarlandı"
