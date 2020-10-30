from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
		front_left = Node(
			package='robominers_serial_interface', 
			namespace='front_right',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[{'motor_name': 'front_right'},
						{'arduino_sn': 'AB0JQ5XM'}]
			)
		rear_left = Node(
			package='robominers_serial_interface', 
			namespace='rear_right',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[{'motor_name': 'rear_right'},
						{'arduino_sn': 'AK08KRS8'}]
			)
		rear_right = Node(
			package='robominers_serial_interface', 
			namespace='rear_left',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[{'motor_name': 'rear_left'},
						{'arduino_sn': 'AC013EU8'}]
			)
		front_right = Node(
			package='robominers_serial_interface', 
			namespace='front_left',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[{'motor_name': 'front_left'},
						{'arduino_sn': 'AK08KMQ8'}]
			)
		return LaunchDescription([
			front_right,
			rear_right,
			rear_left,
			front_left
			])