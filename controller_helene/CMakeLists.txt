cmake_minimum_required(VERSION 3.8)
project(controller_helene)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(helene_msgs REQUIRED)

add_library(controller_helene SHARED src/helene_hardware_interface.cpp)

# specify libraries to link a library or executable target against
ament_target_dependencies(controller_helene
  hardware_interface
  pluginlib
  rclcpp
  controller_interface
  helene_msgs
)

target_include_directories(controller_helene PRIVATE include)

install(
  TARGETS controller_helene
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES helene_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface helene_hardware.xml)

ament_package()