from launch import LaunchDescription
from launch_ros.actions import Node
import pathlib

# from ament_index_python.packages import get_package_share_directory

cwd = str(pathlib.Path(__file__).parents[0])
m0_params = cwd + "/motor0.yaml"
m1_params = cwd + "/motor1.yaml"

print(m0_params)

def generate_launch_description():
	return LaunchDescription([
		config= os.path.join(
			get_package_share_directory('test_python_package'),
			'config',
			'motor0.yaml',
			'motor1.yaml'
			),
		Node(
			package='test_python_package', 
			namespace='motor0',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[config]
			),
		Node(
			package='test_python_package', 
			namespace='motor1',
			executable='serial_interface',
			name='serial_interface',
			output='screen',
			parameters=[config]
			)
		])