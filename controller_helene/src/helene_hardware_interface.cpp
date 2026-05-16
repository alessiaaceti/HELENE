#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include "pluginlib/class_list_macros.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "helene_msgs/msg/joint_position.hpp" 

// include the header for the hardware interface class
namespace controller_helene 
{

class HeleneHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    hw_states_position_.resize(6, 0.0);
    hw_states_velocity_.resize(6, 0.0);
    hw_commands_velocity_.resize(6, 0.0);

    // initialize ROS2 node, publisher and subscriber
    node_ = std::make_shared<rclcpp::Node>("helene_hw_internal_node");
    
    // publisher to send velocity commands to the robot
    pub_ = node_->create_publisher<helene_msgs::msg::JointPosition>("hardware_commands", 10);
    
    // subscriber to receive position updates from the robot
    sub_ = node_->create_subscription<helene_msgs::msg::JointPosition>(
      "hardware_states", 10,
      [this](const helene_msgs::msg::JointPosition::SharedPtr msg) {
        this->angles_msg_ = *msg; // store the latest joint angles
        // also subscribe to velocities if needed, for now we assume they come in the same message
      });

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < 6; i++) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < 6; i++) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
    }
    return command_interfaces;
  }

  hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // process incoming messages (if any)
    rclcpp::spin_some(node_);

    // read position
    hw_states_position_[0] = ((double)angles_msg_.joint1 / 16384.0) * M_PI * 2.0;
    hw_states_position_[1] = ((double)angles_msg_.joint2 / 16384.0) * M_PI * 2.0;
    hw_states_position_[2] = ((double)angles_msg_.joint3 / 16384.0) * M_PI * 2.0;
    hw_states_position_[3] = ((double)angles_msg_.joint4 / 16384.0) * M_PI * 2.0;
    hw_states_position_[4] = ((double)angles_msg_.joint5 / 16384.0) * M_PI * 2.0;
    hw_states_position_[5] = ((double)angles_msg_.joint6 / 16384.0) * M_PI * 2.0;

    // read velocity
    hw_states_velocity_[0] = ((double)velocities_msg_.joint1 / (71583.0 * M_PI * 2.0)) * 40.0;
    hw_states_velocity_[1] = ((double)velocities_msg_.joint2 / (71583.0 * M_PI * 2.0)) * 40.0;
    hw_states_velocity_[2] = ((double)velocities_msg_.joint3 / (71583.0 * M_PI * 2.0)) * 40.0;
    hw_states_velocity_[3] = ((double)velocities_msg_.joint4 / (71583.0 * M_PI * 2.0)) * 40.0;
    hw_states_velocity_[4] = ((double)velocities_msg_.joint5 / (71583.0 * M_PI * 2.0)) * 40.0;
    hw_states_velocity_[5] = ((double)velocities_msg_.joint6 / (71583.0 * M_PI * 2.0)) * 40.0;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // convert and prepare the velocity commands
    velocity_command_msg_.joint1 = hw_commands_velocity_[0] * (71583.0 / M_PI * 2.0) / 40.0;
    velocity_command_msg_.joint2 = hw_commands_velocity_[1] * (71583.0 / M_PI * 2.0) / 40.0;
    velocity_command_msg_.joint3 = hw_commands_velocity_[2] * (71583.0 / M_PI * 2.0) / 40.0;
    velocity_command_msg_.joint4 = hw_commands_velocity_[3] * (71583.0 / M_PI * 2.0) / 40.0;
    velocity_command_msg_.joint5 = hw_commands_velocity_[4] * (71583.0 / M_PI * 2.0) / 40.0;
    velocity_command_msg_.joint6 = hw_commands_velocity_[5] * (71583.0 / M_PI * 2.0) / 40.0;

    // 4. 
    pub_->publish(velocity_command_msg_);
    
    return hardware_interface::return_type::OK;
  }

private:
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  helene_msgs::msg::JointPosition angles_msg_;
  helene_msgs::msg::JointPosition velocities_msg_;
  helene_msgs::msg::JointPosition velocity_command_msg_;

  // 
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<helene_msgs::msg::JointPosition>::SharedPtr pub_;
  rclcpp::Subscription<helene_msgs::msg::JointPosition>::SharedPtr sub_;
};

} // namespace controller_helene

// 5. export the plugin
PLUGINLIB_EXPORT_CLASS(controller_helene::HeleneHardwareInterface, hardware_interface::SystemInterface)