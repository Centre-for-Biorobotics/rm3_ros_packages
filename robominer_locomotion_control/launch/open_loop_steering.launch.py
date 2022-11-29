import os

import launch
import launch_ros

from launch_ros.actions import Node

def generate_launch_description():
    steering_node = launch_ros.actions.Node(
        package='robominer_state_estimation',
        executable='rm3_inverse_kinematics',
        name='rm3_inverse_kinematics',
        output='screen'
        )
    bodyvel_handle = Node(
        package='robominer_locomotion_control',
        executable='kinematics_input_handler',
        name='kinematics_input_handler',
        output='log'
    )

    return launch.LaunchDescription([
        steering_node,
        bodyvel_handle
        ])
