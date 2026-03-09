from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    MAVROS 3 drone — TCP bağlantısı (MAVProxy'ye bağımlı DEĞİL).
    
    SITL portları:
      -I0 (drone1) → TCP 5760
      -I1 (drone2) → TCP 5770
      -I2 (drone3) → TCP 5780
    """
    return LaunchDescription([

        # =====================
        # DRONE 1 - MAVROS (TCP 5760)
        # =====================
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='drone1',
            name='mavros',
            parameters=[{
                'fcu_url': 'tcp://127.0.0.1:5760',
                'tgt_system': 1,
                'tgt_component': 1,
            }],
            output='screen'
        ),

        # =====================
        # DRONE 2 - MAVROS (TCP 5770)
        # =====================
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='drone2',
            name='mavros',
            parameters=[{
                'fcu_url': 'tcp://127.0.0.1:5770',
                'tgt_system': 2,
                'tgt_component': 1,
            }],
            output='screen'
        ),

        # =====================
        # DRONE 3 - MAVROS (TCP 5780)
        # =====================
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='drone3',
            name='mavros',
            parameters=[{
                'fcu_url': 'tcp://127.0.0.1:5780',
                'tgt_system': 3,
                'tgt_component': 1,
            }],
            output='screen'
        ),
    ])
