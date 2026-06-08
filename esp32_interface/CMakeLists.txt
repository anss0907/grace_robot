cmake_minimum_required(VERSION 3.8)
project(esp32_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(grace_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executables
add_executable(esp32_interface_node src/esp32_interface_node.cpp)
add_executable(led_showcase_node src/led_showcase_node.cpp)

# Link dependencies
ament_target_dependencies(esp32_interface_node
  rclcpp
  sensor_msgs
  std_msgs
  grace_msgs
)

ament_target_dependencies(led_showcase_node
  rclcpp
  std_msgs
)

# Install executables
install(TARGETS
  esp32_interface_node
  led_showcase_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/esp32_supabase_pusher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Install rviz config
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
