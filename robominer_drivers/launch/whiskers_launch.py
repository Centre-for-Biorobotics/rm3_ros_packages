from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robominer_drivers',
            executable='tlv493d_tca9548a_whiskers',
            name='whiskers_publisher',
            output='screen',
            parameters=[{'debug_mode': False},{'console_print': True}]
        )
    ])
