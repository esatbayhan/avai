from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='detector',
            name='detection_display',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"is_displaying": True}
            ]
        )
    ])
