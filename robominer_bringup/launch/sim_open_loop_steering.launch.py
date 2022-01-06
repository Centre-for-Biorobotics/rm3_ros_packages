import os

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        # launch.actions.DeclareLaunchArgument('joy_vel', default_value='cmd_vel'),
        # launch.actions.DeclareLaunchArgument('joy_config', default_value='ps3'),
        # launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        # launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
        #     launch.substitutions.TextSubstitution(text=os.path.join(
        #         get_package_share_directory('teleop_twist_joy'), 'config', '')),
        #     joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),

        # launch_ros.actions.Node(
        #     package='joy', executable='joy_node', name='joy_node',
        #     parameters=[{
        #         'dev': joy_dev,
        #         'deadzone': 0.3,
        #         'autorepeat_rate': 20.0,
        #     }]),

         launch_ros.actions.Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen'),
            
    	launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robominer_locomotion_control'),
                'launch',
                'open_loop_steering.launch.py')
        ))
    
        ])
