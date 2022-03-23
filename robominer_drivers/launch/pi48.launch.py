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
        ]
    )

    return LaunchDescription(
        [pi48_imu]
    )