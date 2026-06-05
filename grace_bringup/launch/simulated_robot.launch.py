"""
=============================================================================
Grace Simulated Robot — Main Launch File
=============================================================================

USAGE EXAMPLES:

  1) Base robot (no camera):
     ros2 launch grace_bringup simulated_robot.launch.py

  2) With lightweight RGB-only camera (fast — good for development):
     ros2 launch grace_bringup simulated_robot.launch.py use_camera:=true

  3) With full D435i depth camera (RGBD — heavy on GPU):
     ros2 launch grace_bringup simulated_robot.launch.py use_camera:=true use_depth_camera:=true

  4) Human following (requires camera):
     ros2 launch grace_bringup simulated_robot.launch.py use_camera:=true use_human_following:=true

  5) Human following with depth camera (full detection pipeline):
     ros2 launch grace_bringup simulated_robot.launch.py use_camera:=true use_depth_camera:=true use_human_following:=true

  6) SLAM mode:
     ros2 launch grace_bringup simulated_robot.launch.py use_slam:=true

  7) Custom map:
     ros2 launch grace_bringup simulated_robot.launch.py map_name:=my_map

  8) Empty world:
     ros2 launch grace_bringup simulated_robot.launch.py world_name:=empty

CAMERA TOPICS (both modes publish on real D435i topic names):
  RGB-only mode  (use_camera:=true, use_depth_camera:=false):
    - /camera/camera/color/image_raw                   (RGB image)

  Depth mode  (use_camera:=true, use_depth_camera:=true):
    - /camera/camera/color/image_raw                   (RGB image)
    - /camera/camera/color/camera_info                 (camera intrinsics)
    - /camera/camera/aligned_depth_to_color/image_raw  (aligned depth image)
    - /camera/camera/depth/color/points                (point cloud / PointCloud2)

ARGUMENTS:
  use_camera         (default: false)  — Enable camera in simulation
  use_depth_camera   (default: false)  — Upgrade to full RGBD D435i sensor (GPU heavy)
  use_human_following(default: false)  — Enable human following node (requires use_camera:=true)
  use_slam           (default: false)  — SLAM instead of pre-built map localization
  map_name           (default: small_house) — Map folder in grace_mapping/maps/
  world_name         (default: small_house) — Name of the Gazebo world file to load
=============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam            = LaunchConfiguration("use_slam")
    map_name            = LaunchConfiguration("map_name")
    world_name          = LaunchConfiguration("world_name")
    use_camera          = LaunchConfiguration("use_camera")
    use_depth_camera    = LaunchConfiguration("use_depth_camera")
    use_human_following = LaunchConfiguration("use_human_following")

    use_slam_arg = DeclareLaunchArgument("use_slam", default_value="false")

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house",
        description="Name of map folder in grace_mapping/maps/"
    )

    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="small_house",
        description="Name of the Gazebo world to load from grace_description/worlds/"
    )

    use_camera_arg = DeclareLaunchArgument(
        "use_camera",
        default_value="false",
        description="Enable camera simulation"
    )

    use_depth_camera_arg = DeclareLaunchArgument(
        "use_depth_camera",
        default_value="false",
        description="Use full D435i RGBD camera (GPU heavy). Requires use_camera:=true."
    )

    use_human_following_arg = DeclareLaunchArgument(
        "use_human_following",
        default_value="false",
        description="Enable human following node. Remember to also set use_camera:=true."
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_description"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={
            "use_camera":       use_camera,
            "use_depth_camera": use_depth_camera,
            "world_name":       world_name,
        }.items(),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={"use_sim_time": "True"}.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={"map_name": map_name}.items(),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )

    human_following = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("grace_human_following"),
            "launch",
            "human_following.launch.py"
        ),
        condition=IfCondition(use_human_following),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Image viewer — shows RGB stream whenever camera is enabled
    image_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        arguments=["/camera/camera/color/image_raw"],
        condition=IfCondition(use_camera),
    )

    return LaunchDescription([
        use_slam_arg,
        map_name_arg,
        world_name_arg,
        use_camera_arg,
        use_depth_camera_arg,
        use_human_following_arg,
        gazebo,
        controller,
        joystick,
        localization,
        slam,
        navigation,
        human_following,
        rviz,
        image_view,
    ])