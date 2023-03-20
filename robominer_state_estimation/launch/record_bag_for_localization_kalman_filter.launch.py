import launch


def generate_launch_description():
    # uros_agent = launch.actions.ExecuteProcess(
    #     cmd=[
    #         'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
    #         'udp4',
    #         '--port', '8888',
    #         '-v6',
    #         ],
    #     output='screen'
    #     )

    bag_recording = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',

            '/tf',
            '/tf_static',

            '/cmd_vel_keyboard',
            '/robot_body_vel',

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
            '/camera_jetson/imu'


            # '/whiskers',
            # '/WhiskerPointCloud',

            '/robot_pose',
            '/robot_pose_filtered',

            '/spectrometer_left/spectrometer_frequency_command',
            '/spectrometer_left/spectrometer_calibration',
            '/spectrometer_left/reflectance_reading',
            '/spectrometer_left/fluorescence_reading',
            '/spectrometer_right/spectrometer_frequency_command',
            '/spectrometer_right/spectrometer_calibration',
            '/spectrometer_right/reflectance_reading',
            '/spectrometer_right/fluorescence_reading',

            '/camera_jetson/color/image_raw',
            '/camera_jetson/depth/image_rect_raw',
            '/odom',
            '/map_data',
            '/localization_pose',

            '/scan/filtered',
            '/scan',
            
            ],
        output='screen'
        )

    return launch.LaunchDescription([
        bag_recording,
        # uros_agent
        ])
