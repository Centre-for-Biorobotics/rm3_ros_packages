from launch import LaunchDescription
import os

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
	# teleop_node = launch_ros.actions.Node(
	# 	package='teleop_twist_joy',
	# 	executable='teleop_node',
	# 	name='teleop',
	# 	output='screen'		
	# 	)
	teleop_joy = launch.actions.IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('teleop_twist_joy'),
			'launch',
			'teleop-launch.py')
		))
	return launch.LaunchDescription([
		teleop_joy
		])