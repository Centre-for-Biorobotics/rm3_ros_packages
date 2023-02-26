import launch


def generate_launch_description():
    bag_recording = launch.actions.ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            # '/aruco_detect/field_transforms',
            '/estimated_odom_FDynamics',
            '/pi48_imu/data_raw',
            '/whiskers',
            '/WhiskerPointCloud',
            '/robot_pose'
            '/robot_pose_filtered'
            ],
        output='screen'
        )

    return launch.LaunchDescription([
        bag_recording
        ])
