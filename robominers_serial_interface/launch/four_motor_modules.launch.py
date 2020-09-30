from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
		front_left = Node(
			package='robominers_serial_interface', 
			namespace='motor0',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[{'motor_name': 'front_right'},
						{'arduino_sn': '75338323535351D062A0'}]
			)
		rear_left = Node(
			package='robominers_serial_interface', 
			namespace='motor1',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[{'motor_name': 'rear_right'},
						{'arduino_sn': '757353532383512140'}]
			)
		rear_right = Node(
			package='robominers_serial_interface', 
			namespace='motor2',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[{'motor_name': 'rear_left'},
						{'arduino_sn': '7573535323835121D092'}]
			)
		front_right = Node(
			package='robominers_serial_interface', 
			namespace='motor3',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[{'motor_name': 'front_left'},
						{'arduino_sn': '75735353238351513022'}]
			)
		return LaunchDescription([
			front_right,
			rear_right,
			rear_left,
			front_left
			])