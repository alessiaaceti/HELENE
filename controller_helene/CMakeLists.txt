cmake_minimum_required(VERSION 3.8)
project(controller_helene)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# dipendecies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(helene_msgs REQUIRED)

# library
add_library(helene_hardware_interface SHARED 
  src/helene_hardware_interface.cpp
)

# include directories
target_include_directories(helene_hardware_interface PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# dependencies
ament_target_dependencies(helene_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  controller_interface
  helene_msgs
)

# export plugin description
pluginlib_export_plugin_description_file(hardware_interface helene_hardware.xml)

# install targets and files
install(
  TARGETS helene_hardware_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config launch include
  DESTINATION share/${PROJECT_NAME}
)

# install plugin description file
install(
  FILES helene_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/force_feedback_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()