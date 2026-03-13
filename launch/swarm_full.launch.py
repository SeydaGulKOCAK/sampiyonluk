#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║              swarm_full.launch.py — Tam Sürü Sistemi Başlatma               ║
║              TEKNOFEST 2026 Sürü İHA — my_swarm_pkg                         ║
╚══════════════════════════════════════════════════════════════════════════════╝

Başlatılan bileşenler (sırayla, bağımlılık gecikmeli):
  ① Gazebo         — gz-sim dünya dosyası           [with_gz:=true]
  ② ArduCopter SITL — 3 x sim_vehicle.py (I0/I1/I2) [with_sitl:=true]
  ③ MAVROS          — 3 x mavros_node               [with_mavros:=true]
  ④ Per-drone node'lar (drone1/drone2/drone3):
       drone_interface, local_fsm, intent_coordinator,
       formation_controller, qr_perception,
       collision_avoidance, safety_monitor
  ⑤ GCS node'ları (tek örnek):
       swarm_takeoff, mission_fsm, waypoint_navigator
  ⑥ swarm_teleop    — operatör klavye kontrolü       [with_teleop:=true]

KULLANIM:
  # Tam sim (Gazebo + SITL + MAVROS + tüm node'lar)
  ros2 launch my_swarm_pkg swarm_full.launch.py

  # Sadece ROS2 node'ları (dış SITL/MAVROS zaten çalışıyor)
  ros2 launch my_swarm_pkg swarm_full.launch.py \
      with_gz:=false with_sitl:=false with_mavros:=false

  # Drone sayısını değiştir (geliştirme)
  ros2 launch my_swarm_pkg swarm_full.launch.py swarm_size:=2

ARGÜMANLAR:
  swarm_size      : 3        — Drone sayısı
  with_gz         : true     — Gazebo simülasyonu
  with_sitl       : true     — ArduCopter SITL
  with_mavros     : true     — MAVROS bridge
  with_teleop     : false    — Joystick/klavye operatör kontrolü (Görev-2)
  gz_world        : world_base.sdf — Gazebo dünya dosyası
  log_level       : info     — ROS2 log seviyesi

DRONE KONUMLARI (world_base.sdf'den):
  drone1: HOME_X=0,  HOME_Y=0, HOME_Z=0
  drone2: HOME_X=4,  HOME_Y=0, HOME_Z=0
  drone3: HOME_X=8,  HOME_Y=0, HOME_Z=0

GEOFENCE (safety_monitor için):
  X: [-5, 125]   Y: [-5, 95]   Z: [0.5, 60]
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# ══════════════════════════════════════════════════════════════════════════════
# DRONE KONFİGÜRASYONU
# ══════════════════════════════════════════════════════════════════════════════

# (namespace, sysid, mavros_port, home_x, home_y)
DRONE_CONFIGS = [
    ('drone1', 1, 14550, 0.0,  0.0),
    ('drone2', 2, 14560, 4.0,  0.0),
    ('drone3', 3, 14570, 8.0,  0.0),
]

SWARM_SIZE = len(DRONE_CONFIGS)

# Geofence sınırları (safety_monitor)
FENCE = {
    'FENCE_X_MIN': '-5.0',
    'FENCE_X_MAX': '125.0',
    'FENCE_Y_MIN': '-5.0',
    'FENCE_Y_MAX': '95.0',
    'FENCE_Z_MIN': '0.5',
    'FENCE_Z_MAX': '60.0',
}


def generate_launch_description():

    # ── CycloneDDS: FastDDS shared-memory conflict'ini önler ─────────────
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

    pkg_share = get_package_share_directory('my_swarm_pkg')

    # ── Launch argümanları ─────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('swarm_size',  default_value=str(SWARM_SIZE),
                              description='Aktif drone sayısı'),
        DeclareLaunchArgument('with_gz',     default_value='true',
                              description='Gazebo simülasyonunu başlat'),
        DeclareLaunchArgument('with_sitl',   default_value='true',
                              description='ArduCopter SITL başlat'),
        DeclareLaunchArgument('with_mavros', default_value='true',
                              description='MAVROS bridge başlat'),
        DeclareLaunchArgument('with_teleop', default_value='false',
                              description='Joystick/klavye operatör kontrolü'),
        DeclareLaunchArgument('gz_world',    default_value='world_base.sdf',
                              description='Gazebo dünya dosyası adı'),
        DeclareLaunchArgument('log_level',   default_value='info',
                              description='ROS2 log seviyesi'),
    ]

    # ── ① Gazebo ──────────────────────────────────────────────────────────
    world_path = os.path.join(pkg_share, 'worlds', 'world_base.sdf')

    gz_proc = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_gz')),
        additional_env={
            'GZ_SIM_RESOURCE_PATH': ':'.join([
                os.path.join(pkg_share, 'models'),
                os.path.expanduser('~/gz_ws/src/ardupilot_gazebo/models'),
                os.path.expanduser('~/gz_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/models'),
            ]),
        },
    )

    # ── ② ArduCopter SITL (3 örnek) ───────────────────────────────────────
    sitl_procs = []
    for ns, sysid, port, hx, hy in DRONE_CONFIGS:
        idx = sysid - 1  # I0, I1, I2
        sitl_procs.append(
            TimerAction(
                period=2.0 + idx * 1.0,   # cascade başlatma
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'bash', '-c',
                            f'cd ~/ardupilot && '
                            f'sim_vehicle.py -v ArduCopter '
                            f'--model json '
                            f'-I{idx} --sysid {sysid} '
                            f'--out=udp:127.0.0.1:{port} '
                            f'--no-mavproxy'
                        ],
                        output='log',
                        condition=IfCondition(LaunchConfiguration('with_sitl')),
                    )
                ]
            )
        )

    # ── ③ MAVROS (3 örnek) ─────────────────────────────────────────────────
    mavros_nodes = []
    for ns, sysid, port, hx, hy in DRONE_CONFIGS:
        mavros_nodes.append(
            TimerAction(
                period=25.0 + (sysid - 1) * 2.0,  # SITL hazır olduktan sonra
                actions=[
                    Node(
                        package='mavros',
                        executable='mavros_node',
                        namespace=ns,
                        name='mavros',
                        parameters=[{
                            'fcu_url':    f'udp://:{port}@',
                            'tgt_system': sysid,
                            'tgt_component': 1,
                            'pluginlists_yaml': os.path.join(
                                pkg_share, 'config', 'mavros_pluginlists.yaml'
                            ) if os.path.exists(os.path.join(
                                pkg_share, 'config', 'mavros_pluginlists.yaml'
                            )) else '',
                        }],
                        output='log',
                        condition=IfCondition(LaunchConfiguration('with_mavros')),
                        additional_env={'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'},
                        arguments=['--log-level',
                                   LaunchConfiguration('log_level')],
                    )
                ]
            )
        )

    # ── ④ Per-drone node'ları ──────────────────────────────────────────────
    per_drone_nodes = []
    for ns, sysid, port, hx, hy in DRONE_CONFIGS:
        env = {
            'DRONE_NS':              ns,
            'DRONE_ID':              str(sysid),
            'SWARM_SIZE':            str(SWARM_SIZE),
            'HOME_X':                str(hx),
            'HOME_Y':                str(hy),
            'HOME_Z':                '0.0',
            'RMW_IMPLEMENTATION':    'rmw_cyclonedds_cpp',
            **FENCE,
        }

        delay = 12.0 + (sysid - 1) * 0.2  # MAVROS'tan sonra

        # drone_interface
        per_drone_nodes.append(TimerAction(period=delay, actions=[
            Node(
                package='my_swarm_pkg', executable='drone_interface',
                namespace=ns, name='drone_interface',
                additional_env=env, output='screen',
                parameters=[{'drone_id': sysid}],
                arguments=['--log-level',
                           LaunchConfiguration('log_level')],
            )
        ]))

        # local_fsm
        per_drone_nodes.append(TimerAction(period=delay + 0.1, actions=[
            Node(
                package='my_swarm_pkg', executable='local_fsm',
                namespace=ns, name='local_fsm',
                additional_env=env, output='screen',
                parameters=[{'drone_id': sysid}],
                arguments=['--log-level',
                           LaunchConfiguration('log_level')],
            )
        ]))

        # intent_coordinator
        per_drone_nodes.append(TimerAction(period=delay + 0.2, actions=[
            Node(
                package='my_swarm_pkg', executable='intent_coordinator',
                namespace=ns, name='intent_coordinator',
                additional_env=env, output='screen',
                parameters=[{
                    'drone_id':        sysid,
                    'swarm_size':      SWARM_SIZE,
                    'camera_drone_id': 1,
                }],
                arguments=['--log-level',
                           LaunchConfiguration('log_level')],
            )
        ]))

        # formation_controller
        per_drone_nodes.append(TimerAction(period=delay + 0.3, actions=[
            Node(
                package='my_swarm_pkg', executable='formation_controller',
                namespace=ns, name='formation_controller',
                additional_env=env, output='log',
                parameters=[{
                    'drone_id':      sysid,
                    'swarm_size':    SWARM_SIZE,
                    'formation_type': 'OKBASI',
                    'drone_spacing': 6.0,
                }],
                arguments=['--log-level',
                           LaunchConfiguration('log_level')],
            )
        ]))

        # qr_perception
        per_drone_nodes.append(TimerAction(period=delay + 0.4, actions=[
            Node(
                package='my_swarm_pkg', executable='qr_perception',
                namespace=ns, name='qr_perception',
                additional_env=env, output='log',
                parameters=[{
                    'drone_id': sysid,
                    'camera_topic': f'/{ns}/camera/image_raw',
                }],
                arguments=['--log-level',
                           LaunchConfiguration('log_level')],
            )
        ]))

        # collision_avoidance
        per_drone_nodes.append(TimerAction(period=delay + 0.5, actions=[
            Node(
                package='my_swarm_pkg', executable='collision_avoidance',
                namespace=ns, name='collision_avoidance',
                additional_env=env, output='screen',
                parameters=[{
                    'drone_id':   sysid,
                    'swarm_size': SWARM_SIZE,
                }],
                arguments=['--log-level',
                           LaunchConfiguration('log_level')],
            )
        ]))

        # safety_monitor
        per_drone_nodes.append(TimerAction(period=delay + 0.6, actions=[
            Node(
                package='my_swarm_pkg', executable='safety_monitor',
                namespace=ns, name='safety_monitor',
                additional_env=env, output='screen',
                parameters=[{
                    'drone_id': sysid,
                }],
                arguments=['--log-level',
                           LaunchConfiguration('log_level')],
            )
        ]))

    # ── ⑤ GCS node'ları (tek örnek) ───────────────────────────────────────
    gcs_delay = 20.0
    gcs_env   = {
        'SWARM_SIZE':         str(SWARM_SIZE),
        'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
        **FENCE,
    }

    gcs_nodes = [
        # swarm_takeoff
        TimerAction(period=gcs_delay, actions=[
            Node(
                package='my_swarm_pkg', executable='swarm_takeoff',
                name='swarm_takeoff',
                additional_env=gcs_env, output='screen',
                parameters=[{
                    'swarm_size':    SWARM_SIZE,
                    'takeoff_alt_m': 10.0,
                }],
                arguments=['--log-level',
                           LaunchConfiguration('log_level')],
            )
        ]),

        # waypoint_navigator
        TimerAction(period=gcs_delay + 0.2, actions=[
            Node(
                package='my_swarm_pkg', executable='waypoint_navigator',
                name='waypoint_navigator',
                additional_env=gcs_env, output='screen',
                parameters=[{
                    'swarm_size': SWARM_SIZE,
                }],
                arguments=['--log-level',
                           LaunchConfiguration('log_level')],
            )
        ]),

        # mission_fsm (GCS dashboard — interaktif, screen'e çıktı)
        TimerAction(period=gcs_delay + 0.5, actions=[
            Node(
                package='my_swarm_pkg', executable='mission_fsm',
                name='mission_fsm',
                additional_env=gcs_env, output='screen',
                parameters=[{
                    'swarm_size':        SWARM_SIZE,
                    'auto_start':        True,
                    'auto_start_delay':  10.0,
                }],
                arguments=['--log-level',
                           LaunchConfiguration('log_level')],
            )
        ]),
    ]

    # ── ⑥ swarm_teleop (opsiyonel, Görev-2) ──────────────────────────────
    teleop_node = TimerAction(
        period=gcs_delay + 1.0,
        actions=[
            Node(
                package='my_swarm_pkg', executable='swarm_teleop',
                name='swarm_teleop',
                output='screen',
                parameters=[{
                    'use_keyboard':   True,
                    'drone_spacing':  6.0,
                    'formation_type': 'OKBASI',
                }],
                condition=IfCondition(LaunchConfiguration('with_teleop')),
                arguments=['--log-level',
                           LaunchConfiguration('log_level')],
            )
        ]
    )

    # ── Başlangıç bildirimi ────────────────────────────────────────────────
    banner = LogInfo(msg=(
        '\n'
        '╔══════════════════════════════════════════════════════╗\n'
        '║   TEKNOFEST 2026 — Sürü İHA Sistemi Başlatılıyor    ║\n'
        '╚══════════════════════════════════════════════════════╝\n'
        f'  Swarm size : {SWARM_SIZE} drone\n'
        '  Dünya      : world_base.sdf\n'
        '  QR waypoint: (10,20)→(10,50)→(55,75)→(100,50)→(100,20)→(55,5)\n'
    ))

    # ── LaunchDescription ─────────────────────────────────────────────────
    return LaunchDescription([
        *args,
        banner,
        gz_proc,
        *sitl_procs,
        *mavros_nodes,
        *per_drone_nodes,
        *gcs_nodes,
        teleop_node,
    ])
