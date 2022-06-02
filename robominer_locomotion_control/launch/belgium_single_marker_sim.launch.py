import os

import launch
import launch.actions
import launch.substitutions

from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

test_package_name = 'robominer_locomotion_control'

rviz_path = os.path.join(
    get_package_share_directory(test_package_name),
    'config',
    'belgium_sim.rviz2.rviz'
)
if os.path.exists(rviz_path):
    print(f'rviz path: {rviz_path}')
else:
    print(f'bad rviz config path: {rviz_path}')

params_config = os.path.join(
    get_package_share_directory(test_package_name),
    'config',
    'belgium_cave_params_sim.yaml'
    )
print(f'params path: {params_config}')


def generate_launch_description():
    static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "camera", "camera_link"]
        )

    image_publisher_node = Node(
        package=test_package_name,
        executable='image_publisher',
        name='image_publisher_node',
        output='screen',
        remappings=[
            ("/video_frames", "camera/color/image_raw")
        ],
        parameters = [params_config]
    )

    robot_detect_node = Node(
        package=test_package_name,
        executable='cave_robot_detect',
        name='robot_detector',
        output='screen',
        remappings=[
            ("/camera/image_raw", "camera/color/image_raw")
        ],
        parameters=[params_config]
    )
    rviz_viz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', str(rviz_path)]
    )

    return launch.LaunchDescription([
        static_transform,
        image_publisher_node,
        robot_detect_node,
        rviz_viz
        ])
