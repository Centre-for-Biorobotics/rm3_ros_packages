import launch


def generate_launch_description():
    bag_recording = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/aruco_detect/field_transforms',
            '/aruco_detect/robot_transforms',
            '/camera/color/image_raw',
            '/camera/imu',
            '/tf',
            '/tf_static',
            # '/robot_euler',
            # '/robot_pose',
            '/joy',
            # '/front_left/parameter_events',
            # '/rear_left/parameter_events',
            # '/front_right/parameter_events',
            # '/rear_right/parameter_events',
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
            '/whiskers',
            ],
        output='screen'
        )

    return launch.LaunchDescription([
        bag_recording
        ])