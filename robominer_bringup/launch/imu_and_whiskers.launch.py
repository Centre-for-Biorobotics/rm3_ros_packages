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
    imu_interface = Node(
        package='robominer_drivers',
        executable='bno080_imu',
        name='bno080_imu'
        )
    whiskers = Node(
        package='robominer_drivers',
        executable='tlv493d_tca9548a_whiskers',
        name='tlv493d_tca9548a_whiskers',
        output='log',
        parameters=[{'debug_mode': False}, {'console_print': False}]
        )

    return launch.LaunchDescription([
        imu_interface,
        whiskers,
        ])
