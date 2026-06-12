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

  pubs_.resize(6);
  subs_.resize(6);

  // Inizializza i publisher e subscriber dinamici per ogni giunto micro-ROS
  for (size_t i = 0; i < 6; i++) {
    std::string joint_num = std::to_string(i + 1);
    
    // Cambia qui i nomi dei topic se il tuo firmware ESP32 usa stringhe leggermente diverse
    std::string pub_topic = "joint" + joint_num + "/target_velocity";
    std::string sub_topic = "joint" + joint_num + "/actual_angle";

    // Allocazione del Publisher
    pubs_[i] = node_->create_publisher<std_msgs::msg::Float32>(pub_topic, 10);
    
    // Allocazione del Subscriber usando una lambda function catturando l'indice del giunto [i]
    subs_[i] = node_->create_subscription<std_msgs::msg::Float32>(
      sub_topic, 10,
      [this, i](const std_msgs::msg::Float32::SharedPtr msg) {
        this->hw_states_position_[i] = msg->data;
      });
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HeleneHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }
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
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type HeleneHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Spara i callback in coda per aggiornare hw_states_position_ dai sensori ESP32
  rclcpp::spin_some(node_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HeleneHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Invia i comandi calcolati da MoveIt direttamente a ciascun motore
  for (size_t i = 0; i < 6; i++) {
    std_msgs::msg::Float32 command_msg;
    command_msg.data = hw_commands_position_[i];
    pubs_[i]->publish(command_msg);
  }
  return hardware_interface::return_type::OK;
}

} // namespace controller_helene

PLUGINLIB_EXPORT_CLASS(controller_helene::HeleneHardwareInterface, hardware_interface::SystemInterface)