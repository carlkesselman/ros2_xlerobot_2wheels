cmake_minimum_required(VERSION 3.16)
project(xlerobot_odrive_hardware LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

# The protocol layer has no ROS in it, so the probe tool and the tests can use it alone.
add_library(odrive_usb STATIC src/odrive_usb.cpp)
set_target_properties(odrive_usb PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(odrive_usb PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(odrive_usb PUBLIC PkgConfig::LIBUSB nlohmann_json::nlohmann_json)

add_library(${PROJECT_NAME} SHARED src/odrive_system.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
  odrive_usb
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface xlerobot_odrive_hardware.xml)

# Read-only check from a shell: finds the board, prints firmware, bus voltage, axis states and errors.
add_executable(odrive_probe src/odrive_probe.cpp)
target_link_libraries(odrive_probe odrive_usb)

install(TARGETS ${PROJECT_NAME} odrive_usb
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS odrive_probe RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  # Plain GTest rather than ament_add_gtest: RoboStack's gtest_vendor sources don't match the
  # conda gtest headers on the Mac. Apt Jazzy has libgtest-dev either way.
  find_package(GTest REQUIRED)
  add_executable(test_odrive_usb test/test_odrive_usb.cpp)
  target_link_libraries(test_odrive_usb odrive_usb GTest::gtest_main)
  add_test(NAME test_odrive_usb COMMAND test_odrive_usb)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
