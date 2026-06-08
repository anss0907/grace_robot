import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='esp32_interface',
            executable='esp32_interface_node',
            name='esp32_interface_node',
            output='screen',
        ),
        Node(
            package='esp32_interface',
            executable='esp32_supabase_pusher.py',
            name='supabase_pusher',
            output='screen',
        )
    ])
