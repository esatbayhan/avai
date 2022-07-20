from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controls',
            executable='controller',
            name='optimal_track_1_controller',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    "max_relative_distance_cones": 3.0,

                    "drive_linear_speed_start": 0.3,
                    "drive_linear_speed_max": 1.5,
                    "drive_linear_speed_basis": 1.05,
                    "drive_angular_speed_max": 1.0,

                    "orientation_linear_speed": 0.2,
                    "orientation_angular_speed_start": 0.1,
                    "orientation_angular_speed_max": 0.5,
                    "orientation_angular_speed_basis": 1.1,
                    "orientation_counter_threshold": 30
                }
            ]
        )
    ])
