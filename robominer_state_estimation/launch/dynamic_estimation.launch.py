from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()


    robot_dynamic_odom = Node(
        package="robominer_state_estimation",
        executable="rm3_forward_dynamics",
        name='dynamic_estimation',
        output='screen',
                         )

    complementary_filter = Node(
        # needs imu_tools currently in https://github.com/ccny-ros-pkg/imu_tools
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter',
        output='screen',
        remappings=[
            ("/imu/data_raw", "/pi48_imu/data_raw"),
            ("/imu/data", "/pi48_imu/complementary_data")]
    )

    ld.add_action(robot_dynamic_odom)
    ld.add_action(complementary_filter)
    return ld
