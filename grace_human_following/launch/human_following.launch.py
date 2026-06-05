import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
    )

    human_follower = Node(
        package="grace_human_following",
        executable="human_follower_node",
        name="human_follower",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"follow_distance": 1.0},
            {"max_linear_speed": 0.4},
            {"max_angular_speed": 0.8},
            {"linear_gain": 0.5},
            {"angular_gain": 2.0},
            {"dead_zone": 0.05},
            {"detection_timeout": 3.0},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        human_follower,
    ])
