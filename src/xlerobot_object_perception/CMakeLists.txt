cmake_minimum_required(VERSION 3.8)
project(xlerobot_object_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectObjects.srv"
  DEPENDENCIES vision_msgs
)

ament_python_install_package(objperc)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
