cmake_minimum_required(VERSION 3.8)
project(xlerobot_skill_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SkillResult.msg"
  "msg/ObjectRef.msg"
  "action/NavigateTo.action"
  "action/LookAt.action"
  "action/Detect.action"
  "action/MoveArm.action"
  "action/Approach.action"
  DEPENDENCIES geometry_msgs vision_msgs action_msgs
)

ament_package()
