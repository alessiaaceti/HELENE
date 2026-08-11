#include "helene_hardware_interface.hpp"
#include <algorithm> // Required for std::clamp
#include <cmath>
#include <vector>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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

  // Load velocity limits safely from URDF with fallback defaults
  max_velocity_.assign(6, 2.0); // Conservative default fallback (2.0 rad/s)
  for (size_t i = 0; i < info_.joints.size() && i < 6; i++) {
    auto it = info_.joints[i].parameters.find("max_velocity");
    if (it != info_.joints[i].parameters.end()) {
      try {
        max_velocity_[i] = std::stod(it->second);
      } catch (const std::exception & e) {
        RCLCPP_WARN(rclcpp::get_logger("HeleneHardwareInterface"),
          "Joint %s: invalid max_velocity string (%s), using fallback %.2f",
          info_.joints[i].name.c_str(), it->second.c_str(), max_velocity_[i]);
      }
    } else {
      RCLCPP_WARN(rclcpp::get_logger("HeleneHardwareInterface"),
        "Joint %s: max_velocity not defined in URDF/ros2_control tag, using fallback %.2f",
        info_.joints[i].name.c_str(), max_velocity_[i]);
    }
  }

  // Instantiate background node for asynchronous ROS 2 topic communications
  node_ = std::make_shared<rclcpp::Node>("helene_hw_internal_node");

  // Publisher: Stream joint command profiles directly down to the micro-ROS agent link
  array_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("helene_trajectory_controller/joint_commands", 10);
    
  // Unified Subscriber: Intercept real-time telemetry array streamed by the ESP32 firmware
  sub_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "esp_joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (msg->velocity.size() < 6 || msg->position.size() < 6) return;
      
      for (size_t i = 0; i < 6; i++) {
        this->hw_states_position_[i] = msg->position[i];
        this->hw_states_velocity_[i] = msg->velocity[i];
      }

      this->last_telemetry_stamp_ = this->node_->get_clock()->now();
      this->telemetry_stale_ = false;
      this->initial_state_received_ = true;
    });

  // Subscriber 3: Force-Torque sensor telemetry processing
  sub_meas_ = node_->create_subscription<std_msgs::msg::Float32>(
    "raw_meas", 10,
    [this](const std_msgs::msg::Float32::SharedPtr msg) {
      this->hw_sensor_states_[2] = msg->data; 
    });

  RCLCPP_INFO(rclcpp::get_logger("HeleneHardwareInterface"), "Waiting for initial telemetry from ESP32 (60s timeout for homing sequence)...");
  
  // Wait for initial telemetry frame (60s timeout to allow ESP32 homing)
  auto start_time = std::chrono::steady_clock::now();
  while (!initial_state_received_ && rclcpp::ok()) {
    rclcpp::spin_some(node_);
    
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() > 60) {
      RCLCPP_ERROR(rclcpp::get_logger("HeleneHardwareInterface"), "Timeout (60s): No telemetry data received from ESP32.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  last_telemetry_stamp_ = node_->get_clock()->now();
  
  RCLCPP_INFO(rclcpp::get_logger("HeleneHardwareInterface"), "Telemetry received! Hardware interface initialization completed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HeleneHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset watchdog timestamp on activation to prevent immediate false positive timeouts
  last_telemetry_stamp_ = node_->get_clock()->now();
  telemetry_stale_ = false;
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
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type HeleneHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rclcpp::spin_some(node_);

  // Telemetry watchdog: 1.0s tolerance to absorb micro-ROS serial latency jitter
  auto age = node_->get_clock()->now() - last_telemetry_stamp_;
  if (age > telemetry_timeout_) {
    telemetry_stale_ = true;
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("HeleneHardwareInterface"),
      *node_->get_clock(), 1000,
      "ESP32 telemetry delayed (%.3fs), zeroing output commands for safety", age.seconds());
    // Return OK while flagging telemetry_stale_ to prevent crashing ros2_control_node
    return hardware_interface::return_type::OK;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HeleneHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto command_msg = sensor_msgs::msg::JointState();
  command_msg.header.stamp = node_->get_clock()->now();
  command_msg.velocity.resize(6);

  for (size_t i = 0; i < 6; i++) {
    double v = hw_commands_velocity_[i];

    // If telemetry is stale, send zero velocity safely without killing the controller
    if (telemetry_stale_) {
      v = 0.0;
    } else {
      // Check for non-finite values (NaN/Inf) and clamp within hardware bounds
      if (!std::isfinite(v)) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("HeleneHardwareInterface"),
          *node_->get_clock(), 1000, "Non-finite command (NaN/Inf) detected on joint %zu, zeroed out", i);
        v = 0.0;
      }
      v = std::clamp(v, -max_velocity_[i], max_velocity_[i]);
    }

    command_msg.velocity[i] = v;
  }
  
  array_pub_->publish(command_msg);
  return hardware_interface::return_type::OK;
}

} // namespace controller_helene

PLUGINLIB_EXPORT_CLASS(controller_helene::HeleneHardwareInterface, hardware_interface::SystemInterface)