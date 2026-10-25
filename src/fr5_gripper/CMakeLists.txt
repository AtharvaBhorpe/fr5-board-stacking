cmake_minimum_required(VERSION 3.16)
project(fr5_gripper)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/gripper_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()
