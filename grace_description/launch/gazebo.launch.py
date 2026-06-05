import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    grace_description = get_package_share_directory("grace_description")

    use_camera       = LaunchConfiguration("use_camera")
    use_depth_camera = LaunchConfiguration("use_depth_camera")

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                grace_description, "urdf", "grace.urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="small_house")

    use_camera_arg = DeclareLaunchArgument(
        name="use_camera",
        default_value="false",
        description="Enable camera simulation"
    )

    use_depth_camera_arg = DeclareLaunchArgument(
        name="use_depth_camera",
        default_value="false",
        description="Use full D435i RGBD depth camera (GPU heavy). Requires use_camera:=true."
    )

    world_path = PathJoinSubstitution([
            grace_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(grace_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("grace_description"), 'models')

    gazebo_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition,
            " is_sim:=True",
            " use_camera:=",
            use_camera,
            " use_depth_camera:=",
            use_depth_camera,
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "grace"],
    )

    # ── Base bridge (always active): clock, IMU, LiDAR ──────────────────
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[('/imu', '/imu/out')]
    )

    # ── RGB-only camera bridge (use_camera:=true, use_depth_camera:=false) ──
    # Sensor topic in Ignition: /camera/image_raw  (matches original working config)
    # Remapped to D435i color topic so downstream nodes work on both sim and real hw.
    gz_rgb_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        remappings=[
            ("/camera/image_raw", "/camera/camera/color/image_raw"),
        ],
        condition=IfCondition(PythonExpression([
            "'", use_camera, "' == 'true' and '", use_depth_camera, "' == 'false'"
        ])),
    )

    # ── Depth camera bridge (use_depth_camera:=true) ──────────────────────
    # Gazebo rgbd_camera (topic prefix "camera") publishes:
    #   /camera/image          → gz.msgs.Image
    #   /camera/depth_image    → gz.msgs.Image
    #   /camera/points         → gz.msgs.PointCloudPacked
    #   /camera/camera_info    → gz.msgs.CameraInfo
    #
    # All are remapped to match real D435i topic names so downstream nodes
    # work identically regardless of whether real or simulated hardware is used.
    gz_depth_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # ── Color image ────────────────────────────────────────────────
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            # ── Aligned depth (colour frame) ───────────────────────────────
            "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            # ── Point cloud ────────────────────────────────────────────────
            "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            # ── Camera intrinsics ──────────────────────────────────────────
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            # Color image  →  /camera/camera/color/image_raw
            ("/camera/image",       "/camera/camera/color/image_raw"),
            # Depth image  →  /camera/camera/aligned_depth_to_color/image_raw
            ("/camera/depth_image", "/camera/camera/aligned_depth_to_color/image_raw"),
            # Point cloud  →  /camera/camera/depth/color/points
            ("/camera/points",      "/camera/camera/depth/color/points"),
            # Camera info  →  /camera/camera/color/camera_info
            ("/camera/camera_info", "/camera/camera/color/camera_info"),
        ],
        condition=IfCondition(PythonExpression([
            "'", use_camera, "' == 'true' and '", use_depth_camera, "' == 'true'"
        ])),
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        use_camera_arg,
        use_depth_camera_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        gz_rgb_camera_bridge,
        gz_depth_camera_bridge,
    ])
