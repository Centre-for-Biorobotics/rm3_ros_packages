from launch import LaunchDescription
import os

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    serial_peripherals = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robominer_drivers'),
            'launch',
            'four_motor_modules.launch.py')
        ))
    imu_interface = Node(
        package='robominer_drivers',
        executable='bno080_imu',
        name='bno080_imu',
	remappings=[
		("imu","front_imu")
	]
        )
    temperature_interface = Node(
        package='robominer_drivers',
        executable='temperature_sensor.py',
        name='temperature_sensor'
        )
    steering_stuff = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robominer_locomotion_control'),
            'launch',
            'open_loop_steering.launch.py')
        ))
#    bag_recording = launch.actions.ExecuteProcess(
#        cmd=['ros2', 'bag', 'record', '-a'], 
#        output='screen'
#        )

    return launch.LaunchDescription([
#        bag_recording,
        serial_peripherals,
        imu_interface,
        temperature_interface,
        steering_stuff
        ])
