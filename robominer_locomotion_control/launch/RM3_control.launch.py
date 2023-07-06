#!/usr/bin/python3
"""
RM3 trajectory tracking control launch file.

@author: Walid Remmas
@contact: walid.remmas@ŧaltech.ee
"""

import launch_ros
import launch

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

    rm3_forward_dynamics = launch_ros.actions.Node(
        package='robominer_state_estimation',
        executable='rm3_forward_dynamics',
        name='rm3_forward_dynamics',
        parameters=[{'in_simulation': False}],
        output='screen'
        )

    bag_recording = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            # '/cmd_vel_keyboard',
            '/motor0/motor_rpm_setpoint',
            '/motor1/motor_rpm_setpoint',
            '/motor2/motor_rpm_setpoint',
            '/motor3/motor_rpm_setpoint',
            '/front_right/motor_module',
            '/rear_right/motor_module',
            '/rear_left/motor_module',
            '/front_left/motor_module',
            '/temperature_sensor/temperature',
            '/bno080_imu',
            '/pi48_imu/data_raw',
            # '/pi48_imu/complementary_data',
            '/robot_pose',
            '/robot_pose_filtered',
            '/reference_trajectory',
            '/reference_yaw',
            '/move_cmd_vel',
            '/robot_odom'],
        output='screen'
        )

    return LaunchDescription([
        pilot,
        trajectory_manager,
        bag_recording,
        rm3_forward_dynamics

    ])
