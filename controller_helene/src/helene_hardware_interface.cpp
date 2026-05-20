#include "helene_hardware_interface.hpp"
#include <cmath>
#include "pluginlib/class_list_macros.hpp"

namespace controller_helene 
{
hardware_interface::CallbackReturn HeleneHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  hw_states_position_.resize(6, 0.0);
  hw_states_velocity_.resize(6, 0.0);
  hw_commands_velocity_.resize(6, 0.0);
  hw_sensor_states_.fill(0.0); // Inizializzazione sensore

  node_ = std::make_shared<rclcpp::Node>("helene_hw_internal_node");

  pub_ = node_->create_publisher<helene_msgs::msg::JointPosition>("hardware_commands", 10);
  
  sub_ = node_->create_subscription<helene_msgs::msg::JointPosition>(
    "hardware_states", 10,
    [this](const helene_msgs::msg::JointPosition::SharedPtr msg) {
      this->angles_msg_ = *msg;
      this->velocities_msg_ = *msg;
    });

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HeleneHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // joints
  for (uint i = 0; i < 6; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }
  // sensor (predisposition)
  const std::string sensor_name = "tcp_force_torque_sensor";
  state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "force.x", &hw_sensor_states_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "force.y", &hw_sensor_states_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "force.z", &hw_sensor_states_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "torque.x", &hw_sensor_states_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "torque.y", &hw_sensor_states_[4]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, "torque.z", &hw_sensor_states_[5]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HeleneHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < 6; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type HeleneHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // read logic
  rclcpp::spin_some(node_);

  // update joint states
  hw_states_position_[0] = angles_msg_.joint1;
  hw_states_position_[1] = angles_msg_.joint2;
  hw_states_position_[2] = angles_msg_.joint3;
  hw_states_position_[3] = angles_msg_.joint4;
  hw_states_position_[4] = angles_msg_.joint5;
  hw_states_position_[5] = angles_msg_.joint6;
  
  return hardware_interface::return_type::OK;

}

hardware_interface::return_type HeleneHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // update command message
  velocity_command_msg_.joint1 = hw_commands_velocity_[0];
  velocity_command_msg_.joint2 = hw_commands_velocity_[1];
  velocity_command_msg_.joint3 = hw_commands_velocity_[2];
  velocity_command_msg_.joint4 = hw_commands_velocity_[3];
  velocity_command_msg_.joint5 = hw_commands_velocity_[4];
  velocity_command_msg_.joint6 = hw_commands_velocity_[5];
  // command logic
  pub_->publish(velocity_command_msg_);
  return hardware_interface::return_type::OK;
}

} // namespace controller_helene

PLUGINLIB_EXPORT_CLASS(controller_helene::HeleneHardwareInterface, hardware_interface::SystemInterface)