from launch import LaunchDescription
import os

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
	serial_peripherals = launch.actions.IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('robominers_serial_interface'),
			'launch',
			'four_motor_modules.launch.py')
		))
	steering_stuff = launch.actions.IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('steering'),
			'launch',
			'open_loop_steering.launch.py')
		))
	return launch.LaunchDescription([
		serial_peripherals,
		steering_stuff
		])
