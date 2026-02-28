#!/usr/bin/env python3
"""
Complete launch file for ESP32 IMU visualization in RViz2

Starts:
1. ESP32 interface node (reads and publishes raw IMU data)
2. Static TF transforms (base_footprint -> base_link -> imu1_link, imu2_link)
3. Madgwick filters for both IMUs (computes orientation)
4. RViz2 with pre-configured display

Usage:
  ros2 launch esp32_interface visualize_imu.launch.py
  ros2 launch esp32_interface visualize_imu.launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/grace_esp32',
        description='Serial port for ESP32'
    )
    
    # ESP32 Interface Node
    esp32_node = Node(
        package='esp32_interface',
        executable='esp32_interface_node',
        name='esp32_interface',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
            'wheel_radius': 0.065,
            'imu1_frame_id': 'imu1_link',
            'imu2_frame_id': 'imu2_link',
            'left_wheel_joint': 'left_wheel_joint',
            'right_wheel_joint': 'right_wheel_joint',
        }]
    )
    
    # Static TF: base_footprint -> base_link (5cm above ground)
    tf_footprint_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_footprint_to_base_link',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    # Static TF: base_link -> imu1_link (10cm forward, 5cm up)
    tf_base_to_imu1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link_to_imu1',
        arguments=['0.10', '0', '0.05', '0', '0', '0', 'base_link', 'imu1_link']
    )
    
    # Static TF: base_link -> imu2_link (10cm backward, 5cm up)
    tf_base_to_imu2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link_to_imu2',
        arguments=['-0.10', '0', '0.05', '0', '0', '0', 'base_link', 'imu2_link']
    )
    
    # Madgwick Filter for IMU1 (computes orientation from raw gyro+accel)
    imu1_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu1_madgwick_filter',
        output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'fixed_frame': 'imu1_link',
        }],
        remappings=[
            ('imu/data_raw', '/imu1/data'),
            ('imu/data', '/imu1/data_filtered'),
        ]
    )
    
    # Madgwick Filter for IMU2
    imu2_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu2_madgwick_filter',
        output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'fixed_frame': 'imu2_link',
        }],
        remappings=[
            ('imu/data_raw', '/imu2/data'),
            ('imu/data', '/imu2/data_filtered'),
        ]
    )
    
    # RViz2
    rviz_config = PathJoinSubstitution([
        FindPackageShare('esp32_interface'),
        'rviz',
        'esp32_imu.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    
    return LaunchDescription([
        serial_port_arg,
        esp32_node,
        tf_footprint_to_base,
        tf_base_to_imu1,
        tf_base_to_imu2,
        imu1_filter,
        imu2_filter,
        rviz_node,
    ])
