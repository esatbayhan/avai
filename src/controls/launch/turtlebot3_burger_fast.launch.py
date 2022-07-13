from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controls',
            executable='controller',
            name='burger_controller_fast',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    "drive_linear_speed_start": 0.05,
                    "drive_linear_speed_max": 0.22,
                    "drive_linear_speed_basis": 1.2,
                    "drive_angular_speed_max": 0.5,

                    "orientation_linear_speed": 0.05,
                    "orientation_angular_speed_start": 0.1,
                    "orientation_angular_speed_max": 0.4,
                    "orientation_angular_speed_basis": 1.1
                }
            ]
        )
    ])
