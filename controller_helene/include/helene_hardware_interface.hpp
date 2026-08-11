#ifndef HELENE_HARDWARE_HPP
#define HELENE_HARDWARE_HPP

#include <vector>
#include <array>
#include <chrono>

// Standard ROS 2 Control includes
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp" 
#include "std_msgs/msg/float32.hpp"

namespace controller_helene 
{
class HeleneHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HeleneHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // State flags
  bool initial_state_received_{false};
  bool telemetry_stale_{false};

  // ESP32 telemetry watchdog (1000 ms = 1s serial latency tolerance)
  rclcpp::Time last_telemetry_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration telemetry_timeout_{std::chrono::milliseconds(1000)};

  // Internal buffers for commands, positions, velocities, and limits (6 DOF)
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> max_velocity_;

  // Buffer for 6-axis force-torque sensor data
  std::array<double, 6> hw_sensor_states_; 

  // Internal ROS 2 node for communication thread
  rclcpp::Node::SharedPtr node_;
  
  // Publisher and subscriber matching micro-ROS profile
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr array_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_meas_;
};
} 

#endif // HELENE_HARDWARE_HPP