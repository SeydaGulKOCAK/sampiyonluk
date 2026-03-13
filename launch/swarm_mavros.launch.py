from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # =====================
        # DRONE 1 - MAVROS
        # =====================
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='drone1',
            parameters=[{
                'fcu_url': 'udp://:14550@',
                'tgt_system': 1
            }],
            output='screen'
        ),

        # =====================
        # DRONE 2 - MAVROS
        # =====================
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='drone2',
            parameters=[{
                'fcu_url': 'udp://:14560@',
                'tgt_system': 2
            }],
            output='screen'
        ),

        # =====================
        # DRONE 3 - MAVROS
        # =====================
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='drone3',
            parameters=[{
                'fcu_url': 'udp://:14570@',
                'tgt_system': 3
            }],
            output='screen'
        ),
    ])
