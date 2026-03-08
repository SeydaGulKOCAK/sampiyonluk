from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_swarm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='beyza',
    maintainer_email='beyza@gmail.com',
    description='Swarm MAVROS launch package',
    license='TODO',

    entry_points={
        'console_scripts': [
            'swarm_takeoff = my_swarm_pkg.swarm_takeoff:main',
            'drone_interface = my_swarm_pkg.drone_interface:main',
            'local_fsm = my_swarm_pkg.local_fsm:main',
            'intent_coordinator      = my_swarm_pkg.intent_coordinator:main',
            'formation_controller   = my_swarm_pkg.formation_controller:main',
            'qr_perception          = my_swarm_pkg.qr_perception:main',
            'collision_avoidance    = my_swarm_pkg.collision_avoidance:main',
            'mission_fsm           = my_swarm_pkg.mission_fsm:main',
            'waypoint_navigator    = my_swarm_pkg.waypoint_navigator:main',
            'safety_monitor        = my_swarm_pkg.safety_monitor:main',
            'swarm_teleop          = my_swarm_pkg.swarm_teleop:main',
        ],
    },
)

