from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='test_python_package', 
			namespace='motor1',
			executable='serial_interface',
			name='serial_interface',
			output='screen'
			)
		])