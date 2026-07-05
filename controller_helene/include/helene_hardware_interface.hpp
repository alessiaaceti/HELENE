#ifndef HELENE_HARDWARE_HPP
#define HELENE_HARDWARE_HPP

// Standard ROS 2 Control includes
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp" // Changed to support unified micro-ROS array communication
#include "sensor_msgs/msg/joint_state.hpp" 
#include "std_msgs/msg/float32.hpp"

#include <vector>
#include <array>
#include "std_msgs/msg/float32.hpp"

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
  bool initial_state_received_ = false;
  // Internal buffers for joint positions and velocities (6 DOF)
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  // Buffer for 6-axis force-torque sensor data
  std::array<double, 6> hw_sensor_states_; 

  // ROS 2 internal node for standalone communication thread
  rclcpp::Node::SharedPtr node_;
  
  // Single array publisher and subscriber matching the micro-ROS array profile
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr array_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_meas_;
};
} 

#endif