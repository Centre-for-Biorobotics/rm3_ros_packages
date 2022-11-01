import launch


def generate_launch_description():
    bag_recording = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            # '/aruco_detect/field_transforms',
            '/aruco_detect/robot_tag_transforms',
            '/robot_description',
            # '/camera/color/image_raw',
            # '/camera/imu',
            '/tf',
            # '/tf_static',
            # '/robot_euler',
            # '/robot_pose',
            '/joy',
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
            '/whiskers',
            '/WhiskerPointCloud',
            '/robot_path',
            '/robot_pose',
            '/mapData',
            '/camera/color/image_raw',
            '/spectrometer_left/spectrometer_frequency_command',
            '/spectrometer_left/spectrometer_calibration',
            '/spectrometer_left/reflectance_reading',
            '/spectrometer_left/fluorescence_reading',
            '/spectrometer_right/spectrometer_frequency_command',
            '/spectrometer_right/spectrometer_calibration',
            '/spectrometer_right/reflectance_reading',
            '/spectrometer_right/fluorescence_reading',
            '/mapData',
            '/WhiskerPointCloud',
            '/camera_jetson/color/image_raw',
            '/robot_path'
            ],
        output='screen'
        )

    return launch.LaunchDescription([
        bag_recording
        ])
