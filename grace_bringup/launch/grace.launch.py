"""
=============================================================================
Grace Robot — Unified Headless Launch File (Jetson / Production)
=============================================================================

USAGE EXAMPLES:

  1) Base robot (default map localization):
     ros2 launch grace_bringup grace.launch.py

  2) SLAM mode:
     ros2 launch grace_bringup grace.launch.py use_slam:=true

  3) With Intel RealSense D435i camera + web video streaming:
     ros2 launch grace_bringup grace.launch.py use_camera:=true

  4) With EKF sensor fusion (uses STM32 fused IMU by default):
     ros2 launch grace_bringup grace.launch.py use_ekf:=true

  5) With EKF using ESP32 IMU instead:
     ros2 launch grace_bringup grace.launch.py use_ekf:=true imu_topic:=/imu/out

  6) With Arduino Nano power monitoring:
     ros2 launch grace_bringup grace.launch.py use_nano:=true

  7) Full deployment:
     ros2 launch grace_bringup grace.launch.py use_camera:=true use_ekf:=true use_nano:=true

ARGUMENTS:
  use_slam       (default: false)      — SLAM instead of pre-built map localization
  map_name       (default: small_house)— Map folder in grace_mapping/maps/
  use_rosbridge  (default: true)       — Enable rosbridge_server (port 9090)
  use_camera     (default: false)      — Enable D435i camera + web_video_server
  use_ekf        (default: false)      — Enable EKF sensor fusion localization
  imu_topic      (default: /imu/data)  — IMU topic for EKF (STM32 fused output)
  use_nano       (default: false)      — Enable Arduino Nano interface + Supabase
=============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam       = LaunchConfiguration("use_slam")
    map_name       = LaunchConfiguration("map_name")
    use_rosbridge  = LaunchConfiguration("use_rosbridge")
    use_camera     = LaunchConfiguration("use_camera")
    use_ekf        = LaunchConfiguration("use_ekf")
    imu_topic      = LaunchConfiguration("imu_topic")
    use_nano       = LaunchConfiguration("use_nano")

    # ===================== ARGUMENTS =====================

    args = [
        DeclareLaunchArgument("use_slam", default_value="false"),
        DeclareLaunchArgument("map_name", default_value="small_house",
            description="Name of map folder in grace_mapping/maps/"),
        DeclareLaunchArgument("use_rosbridge", default_value="true",
            description="Enable rosbridge_server for web dashboard (port 9090)"),
        DeclareLaunchArgument("use_camera", default_value="false",
            description="Enable Intel RealSense D435i camera + web_video_server"),
        DeclareLaunchArgument("use_ekf", default_value="false",
            description="Enable EKF sensor fusion localization"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data",
            description="IMU topic for EKF (default: STM32 fused output)"),
        DeclareLaunchArgument("use_nano", default_value="false",
            description="Enable Arduino Nano interface + Supabase pusher"),
    ]

    # ===================== CORE ROBOT =====================

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_firmware"),
            "launch", "hardware_interface.launch.py"),
    )

    laser_driver = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[os.path.join(
            get_package_share_directory("grace_bringup"),
            "config", "rplidar_a2m7.yaml")],
        output="screen"
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_controller"),
            "launch", "controller.launch.py"),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_controller"),
            "launch", "joystick_teleop.launch.py"),
        launch_arguments={"use_sim_time": "False"}.items()
    )

    # ===================== NAVIGATION =====================

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_localization"),
            "launch", "global_localization.launch.py"),
        launch_arguments={"map_name": map_name}.items(),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_mapping"),
            "launch", "slam.launch.py"),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_navigation"),
            "launch", "navigation.launch.py"),
        launch_arguments={"use_sim_time": "False"}.items(),
    )

    # ===================== EKF LOCALIZATION =====================

    local_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_localization"),
            "launch", "local_localization.launch.py"),
        launch_arguments={"imu_topic": imu_topic}.items(),
        condition=IfCondition(use_ekf)
    )

    # ===================== WEB CONNECTIVITY =====================

    rosbridge_server = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        condition=IfCondition(use_rosbridge),
        parameters=[{"port": 9090}],
    )

    web_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        condition=IfCondition(use_camera),
        parameters=[{
            "port": 8081,
            "default_stream_type": "mjpeg",
        }],
    )

    # ===================== INTEL REALSENSE D435i =====================

    realsense_camera = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("realsense2_camera"),
            "launch", "rs_launch.py"),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
        }.items(),
        condition=IfCondition(use_camera)
    )

    # ===================== SENSOR INTERFACES =====================

    # STM32 bringup (serial + IMU fusion + Supabase pusher)
    stm32_bringup = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("stm32_interface"),
            "launch", "stm32_bringup.launch.py"),
    )

    # ESP32 Supabase pusher (env sensors: BME680, PMS5003 → Supabase)
    # Topics are now published by grace_firmware hardware interface
    esp32_supabase_pusher = Node(
        package="esp32_interface",
        executable="esp32_supabase_pusher.py",
        name="esp32_supabase_pusher",
        output="screen",
    )

    # Arduino Nano interface + Supabase pusher (ultrasonic, current, battery)
    nano_bringup = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("arduino_nano_interface"),
            "launch", "supabase_pusher.launch.py"),
        condition=IfCondition(use_nano)
    )

    # ===================== LED STATUS CONTROLLER =====================

    led_controller = Node(
        package="grace_firmware",
        executable="led_status_controller",
        name="led_status_controller",
        output="screen",
    )

    # ===================== LAUNCH =====================

    return LaunchDescription(
        args + [
            # Core robot
            hardware_interface,
            laser_driver,
            controller,
            joystick,
            # Navigation
            localization,
            slam,
            navigation,
            # EKF
            local_localization,
            # Web
            rosbridge_server,
            web_video_server,
            # Camera
            realsense_camera,
            # Sensors
            stm32_bringup,
            esp32_supabase_pusher,
            nano_bringup,
            # LED
            led_controller,
        ]
    )
