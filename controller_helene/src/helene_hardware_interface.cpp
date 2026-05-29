#include "helene_hardware_interface.hpp"
#include <cmath>
#include <vector>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controller_helene 
{
hardware_interface::CallbackReturn HeleneHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state and command vectors (6 DOF)
  hw_states_position_.assign(6, 0.0);
  hw_states_velocity_.assign(6, 0.0);
  hw_commands_position_.assign(6, 0.0);
  hw_sensor_states_.fill(0.0);

  // Create internal node for ROS 2 communication
  node_ = std::make_shared<rclcpp::Node>("helene_hw_internal_node");

  // Publisher for position commands sent to the robot
  pub_ = node_->create_publisher<helene_msgs::msg::JointPosition>("hardware_commands", 10);
  
  // Subscriber for current robot states
  sub_ = node_->create_subscription<helene_msgs::msg::JointPosition>(
    "hardware_states", 10,
    [this](const helene_msgs::msg::JointPosition::SharedPtr msg) {
      this->joint_states_msg_ = *msg;
    });

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HeleneHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Export Position and Velocity state interfaces for each joint
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  // Export Force/Torque sensor interfaces (Predisposition)
  const std::string sensor_name = "tcp_force_torque_sensor";
  std::vector<std::string> axes = {"force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};
  for (size_t i = 0; i < axes.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, axes[i], &hw_sensor_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HeleneHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  // Export POSITION command interface
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
  }
  
  return command_interfaces;
}

hardware_interface::return_type HeleneHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Process incoming subscriber messages
  rclcpp::spin_some(node_);

  // Map message states to internal variables
  hw_states_position_[0] = joint_states_msg_.joint1;
  hw_states_position_[1] = joint_states_msg_.joint2;
  hw_states_position_[2] = joint_states_msg_.joint3;
  hw_states_position_[3] = joint_states_msg_.joint4;
  hw_states_position_[4] = joint_states_msg_.joint5;
  hw_states_position_[5] = joint_states_msg_.joint6;

  // Note: If the message includes velocity, map them here to hw_states_velocity_
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HeleneHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Populate command message with target values from controllers
  position_command_msg_.joint1 = hw_commands_position_[0];
  position_command_msg_.joint2 = hw_commands_position_[1];
  position_command_msg_.joint3 = hw_commands_position_[2];
  position_command_msg_.joint4 = hw_commands_position_[3];
  position_command_msg_.joint5 = hw_commands_position_[4];
  position_command_msg_.joint6 = hw_commands_position_[5];

  // Publish to the real hardware/firmware
  pub_->publish(position_command_msg_);
  
  return hardware_interface::return_type::OK;
}

} // namespace controller_helene

PLUGINLIB_EXPORT_CLASS(controller_helene::HeleneHardwareInterface, hardware_interface::SystemInterface)