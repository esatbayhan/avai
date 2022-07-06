from os.path import expanduser

from launch_ros.actions import Node

from launch import LaunchDescription


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
                    "camera_id": f"{expanduser('~')}/data/test.mp4",
                    "fps": 30
                }
            ]
        )
    ])
