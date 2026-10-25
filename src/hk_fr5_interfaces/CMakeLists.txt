cmake_minimum_required(VERSION 3.10)
project(hk_fr5_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveTo.action"
  "action/MoveDelta.action"
  "srv/GetState.srv"
  "msg/GripperErrorCode.msg"
  "msg/GripperState.msg"
  "action/Grip.action"
  "action/Release.action"
  DEPENDENCIES geometry_msgs sensor_msgs moveit_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
