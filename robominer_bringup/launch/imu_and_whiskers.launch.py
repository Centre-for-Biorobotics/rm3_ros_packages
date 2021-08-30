import launch

from launch_ros.actions import Node


def generate_launch_description():
    imu_interface = Node(
        package='robominer_drivers',
        executable='bno080_imu',
        name='bno080_imu',
        parameters=[{'i2c_address': 0x4A}]  # 0x4A or 0x4B
        )
    whiskers = Node(
        package='robominer_drivers',
        executable='tlv493d_tca9548a_whiskers',
        name='tlv493d_tca9548a_whiskers',
        output='log',
        parameters=[{'debug_mode': False}, {'console_print': False}]
        )

    return launch.LaunchDescription([
        imu_interface,
        whiskers,
        ])
