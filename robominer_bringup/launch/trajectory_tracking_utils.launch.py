import os

import launch

from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    motor_interfaces = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robominer_drivers'),
                'launch',
                'four_motor_modules.launch.py')
        ))
    bno_imu_interface = Node(
        package='robominer_drivers',
        executable='bno080_imu',
        name='bno080_imu',
        parameters=[{'i2c_address': 0x4A}],
        remappings=[("imu","bno080_imu")],
        output='screen'
        )
    pi48_imu_interface = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robominer_drivers'),
                'launch',
                'pi48.launch.py')
        ))
    temperature_interface = Node(
        package='robominer_drivers',
        executable='temperature_sensor.py',
        name='temperature_sensor'
        )

    rm3_inverse_dynamics = launch_ros.actions.Node(
        package='robominer_state_estimation',
        executable='rm3_inverse_dynamics',
        name='rm3_inverse_dynamics',
        output='screen'
        )

    bodyvel_handle = Node(
        package='robominer_locomotion_control',
        executable='kinematics_input_handler',
        name='kinematics_input_handler',
        output='log'
    )


    return launch.LaunchDescription([
        motor_interfaces,
        bno_imu_interface,
        pi48_imu_interface,
        temperature_interface,
        rm3_inverse_dynamics,
        bodyvel_handle
        ])
