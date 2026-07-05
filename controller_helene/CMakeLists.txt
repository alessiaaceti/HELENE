cmake_minimum_required(VERSION 3.8)
project(controller_helene)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ==========================================
# 1. Find Dependencies
# ==========================================
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(helene_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ==========================================
# 2. Hardware Interface Library
# ==========================================
add_library(helene_hardware_interface SHARED 
  src/helene_hardware_interface.cpp
)

target_include_directories(helene_hardware_interface PUBLIC
  include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(helene_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  controller_interface
  helene_msgs
)

# --- MODIFICATO QUI: Dipendenza corretta per la libreria ---
if(TARGET helene_msgs__rosidl_typesupport_cpp)
  add_dependencies(helene_hardware_interface helene_msgs__rosidl_typesupport_cpp)
endif()

# ==========================================
# 3. Executables (Nodes)
# ==========================================

# --- Force Feedback Node ---
add_executable(force_feedback_node src/force_feedback_node.cpp)
target_include_directories(force_feedback_node PRIVATE ${orocos_kdl_INCLUDE_DIRS})
ament_target_dependencies(force_feedback_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  kdl_parser
  helene_msgs
  tf2
  tf2_ros
)

# --- MODIFICATO QUI: Dipendenza corretta per il nodo ---
if(TARGET helene_msgs__rosidl_typesupport_cpp)
  add_dependencies(force_feedback_node helene_msgs__rosidl_typesupport_cpp)
endif()

# --- Mock Force Sensor ---
add_executable(mock_force_sensor src/mock_force_sensor.cpp)
ament_target_dependencies(mock_force_sensor rclcpp geometry_msgs tf2 tf2_ros)

# --- Helene Joystick Teleop ---
add_executable(helene_joystick_teleop src/helene_joystick_teleop.cpp)
ament_target_dependencies(helene_joystick_teleop rclcpp sensor_msgs geometry_msgs)

# --- Helene Force Servo Teleop ---
add_executable(helene_force_servo_teleop src/helene_force_servo_teleop.cpp)
ament_target_dependencies(helene_force_servo_teleop rclcpp geometry_msgs std_msgs)

# ==========================================
# 4. Plugins Export
# ==========================================
pluginlib_export_plugin_description_file(hardware_interface helene_hardware.xml)

# ==========================================
# 5. Install Targets
# ==========================================

# Install Hardware Interface Library
install(
  TARGETS helene_hardware_interface
  EXPORT export_helene_hardware_interface
  DESTINATION lib
)

# Install Node Executables
install(
  TARGETS 
    force_feedback_node
    mock_force_sensor
    helene_joystick_teleop
    helene_force_servo_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# Install Directories & Files
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES helene_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install Python Scripts
#install(PROGRAMS
#  scripts/helene_serial_bridge.py
# DESTINATION lib/${PROJECT_NAME}
#)

# Install Python Scripts
install(PROGRAMS
  scripts/helene_keyboard_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================
# 6. Ament Package
# ==========================================
ament_package()