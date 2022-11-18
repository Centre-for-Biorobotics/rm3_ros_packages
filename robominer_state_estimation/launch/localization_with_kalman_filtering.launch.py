from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    base_path = os.path.realpath(
        get_package_share_directory('robominer_state_estimation'))
    rviz_path = base_path+'/config/config_visualization_localization_filtering.rviz'

    message_formater_node = Node(
        package="robominer_state_estimation",
        executable="messages_preparer_for_filtering",
                                )

    motor2vel_node = Node(
        package="robominer_state_estimation",
        executable="rm3_forward_kinematics",
                         )

    static_transform = Node(package="tf2_ros",
                            executable="static_transform_publisher",
                            arguments=["0.44", "0", "0.01", "0", "0", "0",
                                       "base_link", "front_imu_frame"]
                            )

    static_transform2 = Node(package="tf2_ros",
                             executable="static_transform_publisher",
                             arguments=["0", "0", "0", "0", "0", "0",
                                        "base_link", "imu_frame"]
                             )

    rviz_node = Node(package="rviz2",
                     executable="rviz2",
                     arguments=['-d', str(rviz_path)]
                     )

    ukf_localization = Node(
                        package='robot_localization',
                        executable='ukf_node',
                        name='ukf_filter_node',
                        output='screen',
                        parameters=[os.path.join(get_package_share_directory(
                            "robominer_state_estimation"),
                            'config',
                            'ukf_pose_estimation.yaml')],
                        )

    ld.add_action(rviz_node)
    ld.add_action(static_transform)
    ld.add_action(static_transform2)
    ld.add_action(message_formater_node)
    ld.add_action(motor2vel_node)
    ld.add_action(ukf_localization)
    return ld
