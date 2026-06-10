cmake_minimum_required(VERSION 3.8)
project(stm32_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(grace_msgs REQUIRED)

add_executable(stm32_serial_node src/stm32_serial_node.cpp)
target_include_directories(stm32_serial_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(stm32_serial_node PUBLIC c_std_99 cxx_std_17)  
target_link_libraries(stm32_serial_node serial)
ament_target_dependencies(stm32_serial_node
  rclcpp sensor_msgs std_msgs grace_msgs)

add_executable(imu_fusion_node src/imu_fusion_node.cpp)
target_include_directories(imu_fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_fusion_node PUBLIC c_std_99 cxx_std_17)  
ament_target_dependencies(imu_fusion_node
  rclcpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  stm32_serial_node
  imu_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/stm32_supabase_pusher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
