import os

import launch
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lidar_node = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sicks300_2'),
                'launch',
                'scan_with_filter.launch.py'
            )
        )
    )

    # transform from centre of pi48 (what we consider the centre of the robot)
    # and the top of the lidar housing
    lidartop_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.278", "0", "0.045", "0", "0.26", "0", "robot", "top_laser"])

    # transform from the top of the lidar housing to the middle of the lidar 
    # glass (where we assume the beam comes from)
    lidarbeam_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "-0.033", "0", "0", "0", "top_laser", "base_laser_link"])

    return launch.LaunchDescription([
        lidar_node,
        lidartop_static_transform,
        lidarbeam_static_transform
    ])