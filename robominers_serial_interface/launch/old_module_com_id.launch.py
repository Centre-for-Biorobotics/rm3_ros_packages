from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
	return LaunchDescription([
		Node(
			package='robominers_serial_interface', 
			namespace='motor0',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[{'motor_name': 'front_right'},
						{'arduino_sn': "AK08KM8L"}]
			),
		Node(
			package='robominers_serial_interface', 
			namespace='motor1',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[{'motor_name': 'rear_right'},
						{'arduino_sn': "A6027P6W"}]
			)
		])
	