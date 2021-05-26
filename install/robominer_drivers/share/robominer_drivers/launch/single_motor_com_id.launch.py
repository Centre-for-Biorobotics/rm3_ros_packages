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
			parameters=[{'motor_name': 'rear_right'},
						{'arduino_sn': "AK08KRS8"}]
			)
		# Node(
		# 	package='robominers_serial_interface', 
		# 	namespace='motor0',
		# 	executable='motor_serial_interface',
		# 	name='motor_serial_interface',
		# 	output='screen',
		# 	parameters=[{'motor_name': 'rear_right'},
		# 				{'arduino_sn': "AC013EU8"}]
		# 	)
		])
	
