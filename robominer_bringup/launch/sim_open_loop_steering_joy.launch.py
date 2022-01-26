import os

import launch
import launch_ros
# import launch.actions
# import launch.substitutions
# import launch_ros.actions

from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    teleop_joy = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('teleop_twist_joy'),
                'launch',
                'teleop-launch.py')
        ))
    steering_node = launch_ros.actions.Node(
        package='robominer_locomotion_control',
        executable='open_loop_steering',
        name='open_loop_steering',
        output='screen',
        parameters=[{'on_robot': False},
                    {'which_sim': 'gazebo'}]
        )

    return launch.LaunchDescription([
        teleop_joy,
        steering_node
    ])
