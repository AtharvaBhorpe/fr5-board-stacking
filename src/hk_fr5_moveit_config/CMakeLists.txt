cmake_minimum_required(VERSION 3.10)
project(hk_fr5_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
