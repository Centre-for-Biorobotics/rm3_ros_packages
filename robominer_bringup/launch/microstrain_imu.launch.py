import os

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    _MICROSTRAIN_LAUNCH_FILE = os.path.join(
        get_package_share_directory('microstrain_inertial_driver'), 
        'launch', 
        'microstrain_launch.py')
    _CV7_PARAMS_FILE = os.path.join(
        get_package_share_directory('microstrain_inertial_examples'), 
        'config', 
        'cv7', 
        'cv7.yml')
    
    node_imu_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
        launch_arguments={
            'configure': 'true',
            'activate': 'true',
            'params_file': _CV7_PARAMS_FILE,
            'namespace': '/',
        }.items()
    )

    st_robot_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "cv7_link"
        ]
    )

    return LaunchDescription([
        node_imu_driver,
        st_robot_imu
    ])