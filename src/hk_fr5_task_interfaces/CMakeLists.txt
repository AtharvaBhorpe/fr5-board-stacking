cmake_minimum_required(VERSION 3.10)
project(hk_fr5_task_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SkillCode.msg"
  "msg/Board.msg"
  "msg/Zone.msg"
  "msg/Obstacle.msg"
  "msg/World.msg"
  "msg/Job.msg"
  "action/Pick.action"
  "action/Place.action"
  "action/Home.action"
  "action/WaitClear.action"
  "action/ReleaseSafe.action"
  "srv/AddJob.srv"
  "srv/ListJobs.srv"
  "srv/Control.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
