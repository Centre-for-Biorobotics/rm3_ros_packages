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
    
    forwardDynamics_node = Node(
        package="robominer_state_estimation",
        executable="rm3_forward_dynamics",
        output='screen',
                         )

    static_transform = Node(package="tf2_ros",
                            executable="static_transform_publisher",
                            arguments=["0.0", "0", "0.047", "0", "0", "0",
                                       "base_link_est_ukf", "bno_imu_frame"]
                            )

    static_transform2 = Node(package="tf2_ros",
                             executable="static_transform_publisher",
                             arguments=["0", "0", "0", "0", "0", "0",
                                        "base_link_est_ukf", "pi48_imu_frame"]
                             )
    
    static_transform3 = Node(package="tf2_ros",
                             executable="static_transform_publisher",
                             arguments=["0.39", "0", "-0.0915", "-1.588", "0.0", "-1.571",
                                        "base_link_est_ukf", "camera_jetson_link"]
                             )
    
    static_transform4 = Node(package="tf2_ros",
                             executable="static_transform_publisher",
                             arguments=["0", "0", "0", "0", "0", "0",
                                        "gt_initial_pose", "odom"]
                             )

    static_transform5 = Node(package="tf2_ros",
                             executable="static_transform_publisher",
                             arguments=["0", "0", "0", "0", "0", "0",
                                        "base_link_est_ukf", "robot"]  
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
    ld.add_action(static_transform3)
    ld.add_action(static_transform4)
    ld.add_action(static_transform5)
    ld.add_action(message_formater_node)
    ld.add_action(motor2vel_node)
    ld.add_action(ukf_localization)
    ld.add_action(forwardDynamics_node)
    return ld
