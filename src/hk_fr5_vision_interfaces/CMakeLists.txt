cmake_minimum_required(VERSION 3.10)
project(hk_fr5_vision_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PixelToTable.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
