import os

import launch
import launch.actions
import launch.substitutions

from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    teleop_joy = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('teleop_twist_joy'),
                'launch',
                'teleop-launch.py')
        ))
    steering_stuff = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robominer_locomotion_control'),
                'launch',
                'open_loop_steering.launch.py')
        ))
    return launch.LaunchDescription([
        teleop_joy,
        steering_stuff
        ])
