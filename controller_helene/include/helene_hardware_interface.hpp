#ifndef HELENE_HARDWARE_HPP
#define HELENE_HARDWARE_HPP

// Standard ROS 2 Control includes
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <vector>
#include <array>

namespace controller_helene 
{
class HeleneHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Internal buffers for joint positions and velocities (6 DOF)
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  // Buffer for 6-axis force-torque sensor data
  std::array<double, 6> hw_sensor_states_; 

  // ROS 2 communication infrastructure (Array/Vector per i 6 giunti)
  rclcpp::Node::SharedPtr node_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> pubs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> subs_;
};
} 

#endif