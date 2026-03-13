#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                    swarm_competition.launch.py                              ║
║         TEKNOFEST 2026 — 3 Drone Senkron Kalkış + Runtime QR Yükleme        ║
║                       İnşaallah Başarılı! 🙏                               ║
╚══════════════════════════════════════════════════════════════════════════════╝

🎯 BU LAUNCH DOSYASININ YAPTIĞI:

1️⃣ GAZEBO → Simülasyon dünyası açılır (drone modelleri, QR markerları)
2️⃣ SITL x3 → 3 ArduPilot yazılımsal pilot (arka planda, UDP portlarda)
3️⃣ MAVROS x3 → SITL ile ROS2 arasında köprü
4️⃣ PER-DRONE NODES → drone_interface, local_fsm, intent_coordinator, vb.
5️⃣ GCS NODES → mission_fsm (dashboard), qr_perception (QR algılaması)

💡 SENKRON KALKIŞ:
  - Bully algoritması: drone1 otomatik lider olur
  - intent_coordinator lider → 3 drone'e aynı komut gönderir
  - local_fsm: ARM → TAKEOFF → FLYING (sinkronize)

⚠️ RUNTIME QR YÜKLEMESİ (JÜRİ KOORDİNATLARI):
  - mission_fsm dashboard'dan [m] tuşu → QR koordinat giri ekranı
  - SetQRMap service → qr_perception'a runtime'da waypoint gönder
  - YAML dosyası değiştirilmeye gerek YOK! 

🚀 KULLANIM:
  
  # Terminal 1 - Sistemi Başlat
  cd ~/gz_ws && source install/setup.bash
  ros2 launch my_swarm_pkg swarm_competition.launch.py
  
  # Terminal 2 - Mission FSM Dashboard (birkaç saniye sonra)
  # Otomatik açılacak. Komutlar:
  #   [m] → QR koordinatları gir (jüri verisi)
  #   [s] → TASK1 başlat (3 drone aynı anda kalkar)
  #   [a] → Acil durdur (RTL)
  #   [q] → Çık
"""

import os
import subprocess
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    TimerAction,
    LogInfo,
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import time


def generate_launch_description():
    """
    Launch açıklaması:
    
    DRONE KONFİGÜRASYONU (sabit, ama QR'lar runtime'da değişir):
    
    Drone   Ev Konumu    SITL UDP Port   MAVROS Namespace
    -----   -----------  ---------------  -----------------
    drone1  (0, 0, 0)    14550            /drone1/
    drone2  (4, 0, 0)    14560            /drone2/
    drone3  (8, 0, 0)    14570            /drone3/
    
    Haritalama:
      drone1_id = 1, sysid = 1
      drone2_id = 2, sysid = 2
      drone3_id = 3, sysid = 3
    """

    # ── DRONE KONFİGÜRASYONU ──────────────────────────────────────────────
    # (namespace, sysid, sitl_port, home_x, home_y, home_z)
    DRONE_CONFIGS = [
        ('drone1', 1, 14550, 0.0,  0.0, 0.0),
        ('drone2', 2, 14560, 4.0,  0.0, 0.0),
        ('drone3', 3, 14570, 8.0,  0.0, 0.0),
    ]

    # ── GEOFENCİNG PARAMETRE (Güvenlik) ───────────────────────────────────
    # Yarışma alanının sınırları
    FENCE = {
        'FENCE_X_MIN': '-5.0',
        'FENCE_X_MAX': '125.0',
        'FENCE_Y_MIN': '-5.0',
        'FENCE_Y_MAX': '95.0',
        'FENCE_Z_MIN': '0.5',
        'FENCE_Z_MAX': '60.0',
    }

    # ── ORTAM AYARLARI ────────────────────────────────────────────────────
    # ROS2 loopback (DDS: sadece localhost arasında haberleşe)
    env = {
        'ROS_LOCALHOST_ONLY': '1',
        'CYCLONEDDS_URI': 'file://' + os.path.expanduser(
            '~/gz_ws/src/my_swarm_pkg/config/cyclonedds_localhost.xml'
        ),
        'GZ_SIM_RESOURCE_PATH': ':'.join([
            os.path.expanduser('~/gz_ws/src/my_swarm_pkg/models'),
            os.path.expanduser('~/ardupilot_gazebo/models'),
            '/usr/share/gz/gz-sim8/models',
        ]),
    }

    ws = os.path.expanduser('~/gz_ws')
    pkg_dir = os.path.join(ws, 'src/my_swarm_pkg')

    args = []

    # ═══════════════════════════════════════════════════════════════════════
    # BAŞLANGIC BANNER
    # ═══════════════════════════════════════════════════════════════════════
    args.append(
        LogInfo(msg=[
            '\n╔════════════════════════════════════════════════════════════╗\n'
            '║          🚀 SWARM COMPETITION LAUNCH (3 Drone) 🚀            ║\n'
            '║               İnşaallah Başarılı Uçuş! 🙏                   ║\n'
            '╚════════════════════════════════════════════════════════════╝\n'
            '\n📍 ADIM ADIM NE OLACAK:\n'
            '  1️⃣  Gazebo simülasyon dünyası açılıyor...\n'
            '  2️⃣  3x ArduPilot SITL başlatılıyor (15 saniye bekleme)\n'
            '  3️⃣  3x MAVROS bağlantısı kurulacak\n'
            '  4️⃣  Swarm node\'ları spawn edilecek (drone_interface, local_fsm, ...)\n'
            '  5️⃣  GCS dashboard (mission_fsm) terminal açılacak\n'
            '\n⚠️  JÜRİ KOORDİNATLARI (YARN GÜNÜ):\n'
            '  → mission_fsm dashboard\'da [m] tuşuna basın\n'
            '  → QR koordinatlarını giriniz (x,y,z formatı)\n'
            '  → SetQRMap service otomatik gönderilir\n'
            '  → [s] ile mission başlatın\n'
            '\n🎯 SENKRON KALKIŞ:\n'
            '  → Bully algoritması drone1\'i lider yapar\n'
            '  → intent_coordinator 3 drone\'e aynı komut gönderir\n'
            '  → local_fsm: IDLE → ARMING → FLYING (eşzamanlı)\n\n'
        ])
    )

    # ═══════════════════════════════════════════════════════════════════════
    # 1️⃣ GAZEBO
    # ═══════════════════════════════════════════════════════════════════════
    world_file = os.path.join(pkg_dir, 'worlds', 'world_base.sdf')
    
    args.append(
        LogInfo(msg='🌍 GAZEBO başlatılıyor...')
    )

    args.append(
        ExecuteProcess(
            cmd=[
                'gz', 'sim',
                '-r',  # Paused başlasın (drone\'lar hazır olana kadar)
                world_file
            ],
            output='screen',
            additional_env=env,
        )
    )

    # ═══════════════════════════════════════════════════════════════════════
    # 2️⃣ ArduPilot SITL x3
    # ═══════════════════════════════════════════════════════════════════════
    sitl_procs = []

    for ns, sysid, port, hx, hy, hz in DRONE_CONFIGS:
        idx = sysid - 1  # -I parameter için (0, 1, 2)
        
        args.append(
            LogInfo(msg=f'🛩️  SITL drone{sysid} (port {port}) başlatılıyor...')
        )

        # Timer: SITL'ler sırayla başlasın (boğulma önleme)
        sitl_procs.append(
            TimerAction(
                period=idx * 2.0,  # drone1: 0s, drone2: 2s, drone3: 4s
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'bash', '-c',
                            f'cd ~/ardupilot && '
                            f'sim_vehicle.py -v ArduCopter '
                            f'--model json '
                            f'-I{idx} --sysid {sysid} '
                            f'--out=udp:127.0.0.1:{port} '
                            f'--no-mavproxy '
                            f'--home={hx},{hy},{hz},0'
                        ],
                        output='log',
                        additional_env=env,
                    )
                ]
            )
        )

    args.extend(sitl_procs)

    # Bekleme: SITL'ler hazırlanması için
    args.append(
        LogInfo(msg='⏳ SITL hazırlığı için 15 saniye bekleniyor...')
    )

    # ═══════════════════════════════════════════════════════════════════════
    # 3️⃣ MAVROS x3
    # ═══════════════════════════════════════════════════════════════════════
    mavros_nodes = []

    for ns, sysid, port, hx, hy, hz in DRONE_CONFIGS:
        args.append(
            LogInfo(msg=f'🔌 MAVROS {ns} (sysid={sysid}) kurulacak...')
        )

        mavros_nodes.append(
            TimerAction(
                period=8.0 + (sysid - 1) * 0.5,  # SITL hazır olduktan sonra
                actions=[
                    Node(
                        package='mavros',
                        executable='mavros_node',
                        namespace=ns,
                        parameters=[{
                            'fcu_url': f'udp://:{port}@',
                            'tgt_system': sysid,
                        }],
                        output='log',
                        additional_env=env,
                    )
                ]
            )
        )

    args.extend(mavros_nodes)

    # ═══════════════════════════════════════════════════════════════════════
    # 4️⃣ PER-DRONE NODES (drone_interface, local_fsm, intent_coordinator, ...)
    # ═══════════════════════════════════════════════════════════════════════
    per_drone_nodes = []

    for ns, sysid, port, hx, hy, hz in DRONE_CONFIGS:
        delay = 9.0 + (sysid - 1) * 0.5
        
        env_vars = {
            'DRONE_NS': ns,
            'DRONE_ID': str(sysid),
            'SWARM_SIZE': '3',
            'HOME_X': str(hx),
            'HOME_Y': str(hy),
            'HOME_Z': str(hz),
        }
        env_vars.update(FENCE)
        env_vars.update(env)

        # ── drone_interface (MAVROS köprüsü) ──────────────────────────────
        per_drone_nodes.append(
            TimerAction(
                period=delay,
                actions=[
                    Node(
                        package='my_swarm_pkg',
                        executable='drone_interface',
                        namespace=ns,
                        additional_env=env_vars,
                        output='log',
                    )
                ]
            )
        )

        # ── local_fsm (drone durum makinesi) ──────────────────────────────
        per_drone_nodes.append(
            TimerAction(
                period=delay + 0.1,
                actions=[
                    Node(
                        package='my_swarm_pkg',
                        executable='local_fsm',
                        namespace=ns,
                        additional_env=env_vars,
                        output='log',
                    )
                ]
            )
        )

        # ── intent_coordinator (lider seçimi + QR dağıtımı) ──────────────
        per_drone_nodes.append(
            TimerAction(
                period=delay + 0.2,
                actions=[
                    Node(
                        package='my_swarm_pkg',
                        executable='intent_coordinator',
                        namespace=ns,
                        additional_env=env_vars,
                        output='log',
                    )
                ]
            )
        )

        # ── formation_controller (virtual structure) ──────────────────────
        per_drone_nodes.append(
            TimerAction(
                period=delay + 0.3,
                actions=[
                    Node(
                        package='my_swarm_pkg',
                        executable='formation_controller',
                        namespace=ns,
                        parameters=[{
                            'formation_type': 'OKBASI',
                            'drone_spacing': 6.0,
                        }],
                        additional_env=env_vars,
                        output='log',
                    )
                ]
            )
        )

        # ── collision_avoidance (APF çarpışma önleme) ────────────────────
        per_drone_nodes.append(
            TimerAction(
                period=delay + 0.4,
                actions=[
                    Node(
                        package='my_swarm_pkg',
                        executable='collision_avoidance',
                        namespace=ns,
                        additional_env=env_vars,
                        output='log',
                    )
                ]
            )
        )

        # ── safety_monitor (batarya + geofence + heartbeat) ──────────────
        per_drone_nodes.append(
            TimerAction(
                period=delay + 0.5,
                actions=[
                    Node(
                        package='my_swarm_pkg',
                        executable='safety_monitor',
                        namespace=ns,
                        additional_env=env_vars,
                        output='log',
                    )
                ]
            )
        )

    args.extend(per_drone_nodes)

    # ═══════════════════════════════════════════════════════════════════════
    # 5️⃣ GCS NODES (mission_fsm, qr_perception, waypoint_navigator, ...)
    # ═══════════════════════════════════════════════════════════════════════
    gcs_nodes = []

    # ── qr_perception (QR algılama + runtime map loading) ────────────────
    args.append(
        LogInfo(msg='👁️  QR Perception node\'u kurulacak (runtime map ready)...')
    )

    gcs_nodes.append(
        TimerAction(
            period=10.5,
            actions=[
                Node(
                    package='my_swarm_pkg',
                    executable='qr_perception',
                    parameters=[{
                        'team_id': 'team1',
                        'camera_drone_id': 1,
                        'backup_camera_drone_id': 2,
                        'num_drones': 3,
                        'trigger_radius': 5.0,
                    }],
                    output='screen',
                    additional_env=env,
                )
            ]
        )
    )

    # ── waypoint_navigator (QR→QR rota takibi) ────────────────────────────
    gcs_nodes.append(
        TimerAction(
            period=10.6,
            actions=[
                Node(
                    package='my_swarm_pkg',
                    executable='waypoint_navigator',
                    additional_env=env,
                    output='log',
                )
            ]
        )
    )

    # ── mission_fsm (GCS dashboard + TASK1/TASK2 başlatma) ────────────────
    args.append(
        LogInfo(msg='🎮 MISSION FSM DASHBOARD açılıyor (yeni xterm\'de)...')
    )

    gcs_nodes.append(
        TimerAction(
            period=10.7,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'xterm',
                        '-title', 'Mission FSM Dashboard',
                        '-e', 'bash', '-c',
                        f'source {ws}/install/setup.bash && '
                        f'export ROS_LOCALHOST_ONLY=1 && '
                        f'export CYCLONEDDS_URI={env["CYCLONEDDS_URI"]} && '
                        f'ros2 run my_swarm_pkg mission_fsm'
                    ],
                    output='screen',
                    additional_env=env,
                )
            ]
        )
    )

    args.extend(gcs_nodes)

    # ═══════════════════════════════════════════════════════════════════════
    # KAPANIŞ BANNER
    # ═══════════════════════════════════════════════════════════════════════
    args.append(
        TimerAction(
            period=12.0,
            actions=[
                LogInfo(msg=[
                    '\n╔════════════════════════════════════════════════════════════╗\n'
                    '║              ✅ SİSTEM BAŞLATILDI - İNŞAALLAH! 🙏            ║\n'
                    '╚════════════════════════════════════════════════════════════╝\n'
                    '\n📊 DASHBOARD KOMUTLARI:\n'
                    '  [m]  → Jüri QR Koordinatlarını Gir (runtime setup)\n'
                    '  [s]  → TASK1 Başlat (3 drone senkron kalkar)\n'
                    '  [a]  → Acil Durdur (RTL Mode)\n'
                    '  [d]  → Durum Göster\n'
                    '  [q]  → Çık\n'
                    '\n💡 İPUÇLARİ:\n'
                    '  • Log dosyaları: ~/.swarm_logs/\n'
                    '  • Gazebo 4 saniye sonra oynatmaya başlayabilir\n'
                    '  • Drone konumları /drone{i}/pose topic\'inde\n'
                    '  • Intent mesajları /swarm/intent topic\'inde\n\n'
                ])
            ]
        )
    )

    # ═══════════════════════════════════════════════════════════════════════
    return LaunchDescription(args)
    # ═══════════════════════════════════════════════════════════════════════


if __name__ == '__main__':
    describe = generate_launch_description()
    print(describe)
