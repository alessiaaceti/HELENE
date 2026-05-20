#ifndef HELENE_HARDWARE_HPP
#define HELENE_HARDWARE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <array>
#include "helene_msgs/msg/joint_position.hpp"

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
  // joints variables
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  // force sensor predisposition
  std::array<double, 6> hw_sensor_states_; 

  // messages and ROS2 communication
  helene_msgs::msg::JointPosition angles_msg_;
  helene_msgs::msg::JointPosition velocities_msg_;
  helene_msgs::msg::JointPosition velocity_command_msg_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<helene_msgs::msg::JointPosition>::SharedPtr pub_;
  rclcpp::Subscription<helene_msgs::msg::JointPosition>::SharedPtr sub_;
};
} 

#endif