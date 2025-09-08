import launch


def generate_launch_description():
    bag_recording = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/joy',
            '/robot_body_vel',

            '/motor0/motor_rpm_setpoint',
            '/motor1/motor_rpm_setpoint',
            '/motor2/motor_rpm_setpoint',
            '/motor3/motor_rpm_setpoint',
            '/front_right/motor_module',
            '/rear_right/motor_module',
            '/rear_left/motor_module',
            '/front_left/motor_module',
            
            '/robot_pose',
            '/robot_pose_filtered',

            '/audio',
            '/bno080_imu',
            '/temperature_sensor/temperature',
            '/whiskers',
            ],
        output='screen'
        )

    return launch.LaunchDescription([
        bag_recording
        ])
