cmake_minimum_required(VERSION 3.8)
project(arduino_nano_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Dependencies ----
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(grace_msgs         REQUIRED)

# ---- Include directories ----
include_directories(include)

# ---- C++ node ----
add_executable(arduino_nano_interface_node
  src/arduino_nano_interface_node.cpp
)

ament_target_dependencies(arduino_nano_interface_node
  rclcpp
  sensor_msgs
  std_msgs
  grace_msgs
)

# ---- Install C++ executable ----
install(TARGETS
  arduino_nano_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Install Python scripts ----
install(PROGRAMS
  scripts/nano_supabase_pusher.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Install launch files ----
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# ---- Install firmware (reference only, not compiled by colcon) ----
install(DIRECTORY
  firmware
  DESTINATION share/${PROJECT_NAME}/
)

# ---- Install udev rules (reference — user copies manually) ----
install(DIRECTORY
  udev
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
