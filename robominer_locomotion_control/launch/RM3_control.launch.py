#!/usr/bin/python3
"""
RM3 trajectory tracking control launch file.

@author: Walid Remmas
@contact: walid.remmas@ŧaltech.ee
"""

import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    trajectory_manager = launch_ros.actions.Node(
        package='robominer_locomotion_control',
        executable='trajectory_manager',
        name='trajectory_manager',
        output='screen'
        )

    pilot = launch_ros.actions.Node(
        package='robominer_locomotion_control',
        executable='pilot',
        name='pilot',
        parameters=[{'in_simulation': False}],
        output='screen'
        )

    return LaunchDescription([
        pilot,
        trajectory_manager,

    ])
