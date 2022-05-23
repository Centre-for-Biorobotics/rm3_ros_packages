import os

import launch
# import launch_ros

from launch_ros.actions import Node

def generate_launch_description():
    move_action_client = Node(
        package='robominer_bt_bridge',
        executable='move_action_client',
        name='move_action_client',
        output='screen'
        )
    move_action_server = Node(
        package='robominer_bt_bridge',
        executable='move_action_server',
        name='move_action_server',
        output='screen'
        )

    return launch.LaunchDescription([
        move_action_client,
        move_action_server
        ])