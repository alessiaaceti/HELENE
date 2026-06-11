#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class HeleneForceServoTeleop : public rclcpp::Node {
public:
    HeleneForceServoTeleop() : Node("helene_force_servo_teleop") {
        
        // --- PARAMETERS ---
        this->declare_parameter<double>("force_deadzone", 1.0);         // Newtons
        this->declare_parameter<double>("admittance_gain", 0.005);      // Scale factor: force -> velocity
        this->declare_parameter<std::string>("frame_id", "base_link");  // Reference frame for the command

        this->get_parameter("force_deadzone", deadzone_);
        this->get_parameter("admittance_gain", admittance_gain_);
        this->get_parameter("frame_id", frame_id_);

        // --- SUBSCRIPTIONS AND PUBLISHERS ---
        // Reads from the force/torque sensor (same topic used by your mock/real node)
        force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force_torque_sensor", 10,
            std::bind(&HeleneForceServoTeleop::force_callback, this, std::placeholders::_1));
        
        // Publishes toward MoveIt Servo
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);
        
        // Debug publisher for the filtered force
        filtered_force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/helene/filtered_force_z", 10);

        // Filter variables initialization
        bias_acquired_ = false;
        bias_count_ = 0;
        z_bias_ = 0.0;
        first_msg_ = true;
        a1_ = 0.993; b0_ = 0.668; b1_ = -0.662; 

        RCLCPP_INFO(this->get_logger(), "Force Teleop Node for MoveIt Servo started. Calibrating tare...");
    }

private:
    void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        double fx = msg->wrench.force.x;
        double fy = msg->wrench.force.y;
        double z_raw = msg->wrench.force.z;
        
        // --- 1. TARE CALIBRATION ---
        if (!bias_acquired_) {
            z_bias_ += z_raw;
            bias_count_++;
            if (bias_count_ >= 100) {
                z_bias_ /= 100.0;
                bias_acquired_ = true;
                RCLCPP_INFO(this->get_logger(), "Tare calibration completed. Sensor ready! Z Bias: %.2f", z_bias_);
            }
            return;
        }

        // --- 2. IIR FILTERING ---
        double z_meas = z_raw - z_bias_;
        if (first_msg_) {
            z_meas_prev_ = z_meas; z_comp_prev_ = z_meas;
            first_msg_ = false;
        }

        // IIR filter to compensate for PLA creep
        double z_comp = (a1_ * z_comp_prev_) + (b0_ * z_meas) + (b1_ * z_meas_prev_);
        z_meas_prev_ = z_meas; z_comp_prev_ = z_comp;

        // Publish filtered force for plotjuggler/debugging
        std_msgs::msg::Float64 filtered_msg;
        filtered_msg.data = z_comp;
        filtered_force_pub_->publish(filtered_msg);

        // --- 3. CONVERSION TO TWIST FOR MOVEIT SERVO ---
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = this->now();
        twist_msg->header.frame_id = frame_id_;

        // Apply pure admittance (v = gain * F) if deadzone threshold is exceeded
        twist_msg->twist.linear.x = (std::abs(fx) > deadzone_) ? fx * admittance_gain_ : 0.0;
        twist_msg->twist.linear.y = (std::abs(fy) > deadzone_) ? fy * admittance_gain_ : 0.0;
        twist_msg->twist.linear.z = (std::abs(z_comp) > deadzone_) ? z_comp * admittance_gain_ : 0.0;

        // Send the Cartesian velocity command to MoveIt Servo
        twist_pub_->publish(std::move(twist_msg));
    }

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_force_pub_;
    
    // Calibration and filtering variables
    double z_bias_;
    bool bias_acquired_;
    int bias_count_;
    bool first_msg_;
    double z_meas_prev_, z_comp_prev_, a1_, b0_, b1_;
    
    // Control parameters
    double deadzone_;
    double admittance_gain_;
    std::string frame_id_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeleneForceServoTeleop>());
    rclcpp::shutdown();
    return 0;
}