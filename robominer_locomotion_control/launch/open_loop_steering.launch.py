import os

import launch
import launch_ros

def generate_launch_description():
    steering_node = launch_ros.actions.Node(
        package='robominer_locomotion_control',
        executable='open_loop_steering',
        name='open_loop_steering',
        output='screen',
        parameters=[{'on_robot': True}]
        )

    return launch.LaunchDescription([
        steering_node
        ])
