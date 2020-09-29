from launch import LaunchDescription
import os

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
	steering_node = launch_ros.actions.Node(
		package='steering', 
		executable='open_loop_steering',
		name='open_loop_steering',
		output='screen',
		remappings=[('/cmd_vel', '/turtle1/cmd_vel')]		
		)
	turtlesim_node = launch_ros.actions.Node(
		package='turtlesim',
		executable='turtle_teleop_key',
		name='teleop',
		output='screen'
		)
	return launch.LaunchDescription([
		steering_node,
		turtlesim_node
		])