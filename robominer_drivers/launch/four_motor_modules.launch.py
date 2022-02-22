from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    front_right = Node(
        package='robominer_drivers',
        namespace='front_right',
        executable='motor_serial_interface.py',
        name='motor_serial_interface',
        output='screen',
        parameters=[{'motor_name': 'front_right'},
                    {'arduino_sn': 'AB0LB3F4'}]
        )
    rear_right = Node(
        package='robominer_drivers',
        namespace='rear_right',
        executable='motor_serial_interface.py',
        name='motor_serial_interface',
        output='screen',
        parameters=[{'motor_name': 'rear_right'},
                    {'arduino_sn': 'AB0LB3F2'}]
        )
    rear_left = Node(
        package='robominer_drivers',
        namespace='rear_left',
        executable='motor_serial_interface.py',
        name='motor_serial_interface',
        output='screen',
        parameters=[{'motor_name': 'rear_left'},
                    {'arduino_sn': 'AB0LB3F3'}]
        )
    front_left = Node(
        package='robominer_drivers',
        namespace='front_left',
        executable='motor_serial_interface.py',
        name='motor_serial_interface',
        output='screen',
        parameters=[{'motor_name': 'front_left'},
                    {'arduino_sn': 'AQ00PUPC'}]
        )
    return LaunchDescription([
        front_right,
        rear_right,
        rear_left,
        front_left
        ])
