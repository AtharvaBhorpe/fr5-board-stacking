cmake_minimum_required(VERSION 3.10)
project(hk_fr5_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveTo.action"
  "action/MoveDelta.action"
  "srv/GetState.srv"
  DEPENDENCIES geometry_msgs sensor_msgs moveit_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
