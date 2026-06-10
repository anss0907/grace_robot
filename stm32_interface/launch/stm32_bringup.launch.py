import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stm32_interface',
            executable='stm32_serial_node',
            name='stm32_serial_node',
            output='screen',
            parameters=[
                {'port': '/dev/grace_stm32'},
                {'baud': 921600}
            ]
        ),
        Node(
            package='stm32_interface',
            executable='imu_fusion_node',
            name='imu_fusion_node',
            output='screen'
        ),
        Node(
            package='stm32_interface',
            executable='stm32_supabase_pusher.py',
            name='stm32_supabase_pusher',
            output='screen'
        )
    ])
