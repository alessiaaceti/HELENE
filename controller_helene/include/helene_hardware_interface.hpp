#ifndef HELENE_HARDWARE_HPP
#define HELENE_HARDWARE_HPP

// Standard ROS 2 Control includes
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "helene_msgs/msg/joint_position.hpp"

#include <vector>
#include <array>

namespace controller_helene 
{
class HeleneHardwareInterface : public hardware_interface::SystemInterface
{
public:
  // Lifecycle methods for the hardware interface
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  
  // Methods to export interfaces to the Resource Manager
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  // Real-time read and write loops
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Internal buffers for joint positions and velocities (6 DOF)
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  // Buffer for 6-axis force-torque sensor data (FX, FY, FZ, TX, TY, TZ)
  std::array<double, 6> hw_sensor_states_; 

  // ROS 2 message containers
  helene_msgs::msg::JointPosition joint_states_msg_;
  helene_msgs::msg::JointPosition position_command_msg_;

  // ROS 2 communication infrastructure
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<helene_msgs::msg::JointPosition>::SharedPtr pub_;
  rclcpp::Subscription<helene_msgs::msg::JointPosition>::SharedPtr sub_;
};
} 

#endif