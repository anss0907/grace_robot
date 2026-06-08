from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/grace_nano",
        description="Serial port for the Arduino Nano"
    )

    nano_node = Node(
        package="arduino_nano_interface",
        executable="arduino_nano_interface_node",
        name="arduino_nano_interface_node",
        output="screen",
        parameters=[{
            "serial_port":        LaunchConfiguration("serial_port"),
            "baud_rate":          115200,
            "enable_csv_logging": True,
        }]
    )

    supabase_node = Node(
        package="arduino_nano_interface",
        executable="nano_supabase_pusher.py",
        name="nano_supabase_pusher",
        output="screen",
    )

    return LaunchDescription([
        serial_port_arg,
        nano_node,
        supabase_node,
    ])
