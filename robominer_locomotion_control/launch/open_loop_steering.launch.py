import os

import launch


def generate_launch_description():
    steering_node = launch_ros.actions.Node(
        package='robominer_locomotion_control',
        executable='open_loop_steering',
        name='open_loop_steering',
        output='screen'
        )

    return launch.LaunchDescription([
        steering_node
        ])
