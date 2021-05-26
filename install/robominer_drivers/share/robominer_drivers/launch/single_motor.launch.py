from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
	return LaunchDescription([
		Node(
			package='robominer_drivers', 
			namespace='motor0',
			executable='motor_serial_interface.py',
			name='motor_serial_interface',
			output='screen',
			parameters=[{'motor_name': 'front_right'},
						{'arduino_sn': '75338323535351D062A0'}]			)
		])
	