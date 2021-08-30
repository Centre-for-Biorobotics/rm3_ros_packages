import launch


def generate_launch_description():
    bag_recording = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen'
        )

    return launch.LaunchDescription([
        bag_recording
        ])
