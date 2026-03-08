from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([

        # =====================
        # DRONE 1
        # =====================
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'cd ~/ardupilot && '
                'sim_vehicle.py -v ArduCopter '
                '--model json '
                '-I0 --sysid 1 '
                '--out=udp:127.0.0.1:14550'
            ],
            output='screen'
        ),

        # =====================
        # DRONE 2
        # =====================
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'cd ~/ardupilot && '
                'sim_vehicle.py -v ArduCopter '
                '--model json '
                '-I1 --sysid 2 '
                '--out=udp:127.0.0.1:14560'
            ],
            output='screen'
        ),

        # =====================
        # DRONE 3
        # =====================
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'cd ~/ardupilot && '
                'sim_vehicle.py -v ArduCopter '
                '--model json '
                '-I2 --sysid 3 '
                '--out=udp:127.0.0.1:14570'
            ],
            output='screen'
        ),
    ])


