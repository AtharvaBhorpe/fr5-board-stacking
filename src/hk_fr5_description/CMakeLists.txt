cmake_minimum_required(VERSION 3.10)
project(hk_fr5_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config meshes launch rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/joint_sliders.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
