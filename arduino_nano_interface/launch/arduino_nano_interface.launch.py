from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/grace_nano",
        description="Serial port for the Arduino Nano (udev symlink preferred)"
    )

    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate",
        default_value="115200",
        description="Baud rate for the Arduino Nano serial connection"
    )

    enable_csv_arg = DeclareLaunchArgument(
        "enable_csv_logging",
        default_value="true",
        description="Enable CSV logging of gas and power data"
    )

    nano_node = Node(
        package="arduino_nano_interface",
        executable="arduino_nano_interface_node",
        name="arduino_nano_interface_node",
        output="screen",
        parameters=[{
            "serial_port":        LaunchConfiguration("serial_port"),
            "baud_rate":          LaunchConfiguration("baud_rate"),
            "enable_csv_logging": LaunchConfiguration("enable_csv_logging"),
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        enable_csv_arg,
        nano_node,
    ])
