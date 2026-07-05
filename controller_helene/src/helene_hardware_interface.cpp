#include "helene_hardware_interface.hpp"
#include <cmath>
#include <vector>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp" // Switched to native JointState message type
#include "std_msgs/msg/float32.hpp"

namespace controller_helene 
{

hardware_interface::CallbackReturn HeleneHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize internal state vectors and command buffers
  hw_states_position_.assign(6, 0.0);
  hw_states_velocity_.assign(6, 0.0);
  hw_commands_velocity_.assign(6, 0.0);
  hw_sensor_states_.fill(0.0);

  // Instantiate background node for asynchronous ROS 2 topic communications
  node_ = std::make_shared<rclcpp::Node>("helene_hw_internal_node");

  // Publisher: Stream joint command profiles directly down to the micro-ROS agent link
  array_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("helene_trajectory_controller/joint_commands", 10);
    
  // Unified Subscriber: Intercept real-time telemetry array streamed by the ESP32 firmware
  sub_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "esp_joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      // Defensive length assertion check to prevent segmentation faults
      if (msg->velocity.size() < 6 || msg->position.size() < 6) return;
      
      // Direct assignment map: ESP32 already handles the physical conversion into radians and rad/s
      for (size_t i = 0; i < 6; i++) {
        this->hw_states_position_[i] = msg->position[i];
        this->hw_states_velocity_[i] = msg->velocity[i];
      }

      // Signal that the first valid telemetry frame from the hardware has been caught
      this->initial_state_received_ = true;
    });

  // Subscriber 3: Force-Torque sensor telemetry processing
  sub_meas_ = node_->create_subscription<std_msgs::msg::Float32>(
    "raw_meas", 10,
    [this](const std_msgs::msg::Float32::SharedPtr msg) {
      // Assign the single-axis analog force data to the Z-axis sensor array index
      this->hw_sensor_states_[2] = msg->data; 
    });

  RCLCPP_INFO(rclcpp::get_logger("HeleneHardwareInterface"), "Waiting for initial telemetry from ESP32...");
  
  // Safety timeout (e.g., 10 seconds) to prevent blocking the terminal indefinitely if the ESP32 is offline
  auto start_time = std::chrono::steady_clock::now();
  while (!initial_state_received_ && rclcpp::ok()) {
    rclcpp::spin_some(node_);
    
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() > 10) {
      RCLCPP_ERROR(rclcpp::get_logger("HeleneHardwareInterface"), "Timeout: No data received from ESP32.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  
  RCLCPP_INFO(rclcpp::get_logger("HeleneHardwareInterface"), "Telemetry received! Controller alignment completed.");
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
  
  // Set up multi-axis structural frames for the end-effector force-torque observer
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
    // Export only the velocity interface to match the trajectory execution setup
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type HeleneHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Flush the subscription queues to process lambda callbacks and update telemetry state parameters
  rclcpp::spin_some(node_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HeleneHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto command_msg = sensor_msgs::msg::JointState();
  command_msg.header.stamp = node_->get_clock()->now();
  command_msg.velocity.resize(6);

  // Map the reference velocities calculated by the MoveIt pipeline directly into the message array
  for (size_t i = 0; i < 6; i++) {
    command_msg.velocity[i] = hw_commands_velocity_[i];
  }
  
  // Dispatch the synchronized velocity trajectory command package downstream over the serial bridge
  array_pub_->publish(command_msg);
  return hardware_interface::return_type::OK;
}

} // namespace controller_helene

PLUGINLIB_EXPORT_CLASS(controller_helene::HeleneHardwareInterface, hardware_interface::SystemInterface)