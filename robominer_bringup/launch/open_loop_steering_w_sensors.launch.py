import os

import launch

from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    serial_peripherals = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robominer_drivers'),
                'launch',
                'four_motor_modules.launch.py')
        ))
    imu_interface = Node(
        package='robominer_drivers',
        executable='bno080_imu',
        name='front_bno080_imu',
	parameters=[{'i2c_address': 0x4A}],
        remappings=[
            ("imu", "front_imu")
            ]
        )
    temperature_interface = Node(
        package='robominer_drivers',
        executable='temperature_sensor.py',
        name='temperature_sensor'
        )
    steering_stuff = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robominer_locomotion_control'),
                'launch',
                'open_loop_steering.launch.py')
        ))

    return launch.LaunchDescription([
        serial_peripherals,
        imu_interface,
        temperature_interface,
        steering_stuff
        ])
