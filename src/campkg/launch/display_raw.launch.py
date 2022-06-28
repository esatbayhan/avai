from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='campkg',
            executable='display',
            name='display_raw',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'is_saving': False,
                    "topic_name": "frames_raw",
                    "is_compressed": False
                }
            ]
        )
    ])
