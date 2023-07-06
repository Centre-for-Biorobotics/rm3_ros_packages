import launch


def generate_launch_description():
    bag_recording = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/cmd_vel_keyboard',
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
            '/pi48_imu/complementary_data',
            '/robot_pose',
            '/robot_pose_filtered',
            ],
        output='screen'
        )

    return launch.LaunchDescription([
        bag_recording
        ])
