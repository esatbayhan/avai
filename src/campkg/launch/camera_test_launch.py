from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='campkg',
            executable='camera',
            name='camera_test',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'camera_id': '/home/ubuntu/Downloads/test.mp4',
                    "fps": 30
                }
            ]
        )
    ])
