from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pi48_imu = Node(
        package='robominer_drivers',
        executable='pi48_interface.py',
        name='pi48',
        output='screen',
        parameters=[
            {'device_name': 'pi48'},
            {'device_hwid': '0403:6010'}
        ],
        remappings=[("/imu/data_raw", "/pi48_imu/data_raw")]
    )
    complementary_filter = Node(
        # needs imu_tools currently in https://github.com/ccny-ros-pkg/imu_tools
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter',
        output='screen',
        remappings=[
            ("/imu/data_raw", "/pi48_imu/data_raw"),
            ("/imu/data", "/pi48_imu/data")]
    )
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ["0", "0", "0", "0", "0", "0", "map", "pi48"]
    )

    return LaunchDescription(
        [pi48_imu,
        complementary_filter,
        static_transform
        ]
    )