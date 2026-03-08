#!/usr/bin/env bash
# ============================================================
#  swarm_env.sh  —  Sürü sistemi ortam değişkenleri
#  Her terminalde:  source ~/gz_ws/src/my_swarm_pkg/config/swarm_env.sh
# ============================================================

# ── ROS2 workspace ──────────────────────────────────────────
source /opt/ros/humble/setup.bash
source ~/gz_ws/install/setup.bash

# ── DDS: loopback-only (Wi-Fi kapalıyken multicast hatası önleme) ──
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file:///home/seyda/gz_ws/src/my_swarm_pkg/config/cyclonedds_localhost.xml"

# ── Gazebo model yolları ─────────────────────────────────────
export GZ_SIM_RESOURCE_PATH=\
"/home/seyda/gz_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/models:"\
"/home/seyda/gz_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/worlds:"\
"/home/seyda/ardupilot_gazebo/models:"\
"/home/seyda/ardupilot_gazebo/worlds"

# ── ArduPilot SITL ───────────────────────────────────────────
export PATH="$PATH:$HOME/ardupilot/Tools/autotest"

# ── Swarm parametreleri ──────────────────────────────────────
export SWARM_SIZE=3

echo "✅ Swarm ortamı yüklendi (ROS_LOCALHOST_ONLY=1)"
echo "   DDS: CycloneDDS loopback"
echo "   GZ_SIM_RESOURCE_PATH ayarlandı"
