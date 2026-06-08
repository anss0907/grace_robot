cmake_minimum_required(VERSION 3.8)
project(grace_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EnvironmentData.msg"
  "msg/AirQuality.msg"
  "msg/UltrasonicScan.msg"
  "msg/GasReadings.msg"
  "msg/PowerMonitor.msg"
  "msg/ChargerRelay.msg"
  "action/Fibonacci.action"
  "srv/AddTwoInts.srv"
  "srv/GetTransform.srv"
  DEPENDENCIES geometry_msgs std_msgs
)


ament_package()