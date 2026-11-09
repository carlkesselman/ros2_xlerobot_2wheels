cmake_minimum_required(VERSION 3.8)
project(xlerobot_object_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectObjects.srv"
  "srv/LocateInWrist.srv"
  DEPENDENCIES vision_msgs geometry_msgs
)

ament_python_install_package(objperc)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
