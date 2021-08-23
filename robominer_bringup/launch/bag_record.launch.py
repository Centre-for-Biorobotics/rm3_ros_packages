from launch import LaunchDescription
import os

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bag_recording = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen'
        )

    return launch.LaunchDescription([
        bag_recording
        ])
