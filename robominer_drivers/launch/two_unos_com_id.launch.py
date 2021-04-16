from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
	return LaunchDescription([
		Node(
			package='robominer_drivers', 
			namespace='motor0',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[{'motor_name': 'front_right'},
						{'arduino_sn': "75338323535351D062A0"}]
			),
		Node(
			package='robominer_drivers', 
			namespace='motor1',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[{'motor_name': 'rear_right'},
						{'arduino_sn': "757353532383512140"}]
			)
		])
	