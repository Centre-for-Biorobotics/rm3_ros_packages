import os

import launch

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
    steering_stuff = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robominer_locomotion_control'),
                'launch',
                'open_loop_steering.launch.py')
        ))
    return launch.LaunchDescription([
        serial_peripherals,
        steering_stuff
        ])
