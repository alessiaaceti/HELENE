#include <chrono>
#include <memory>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class HeleneForceServoTeleop : public rclcpp::Node {
public:
    HeleneForceServoTeleop() : Node("helene_force_servo_teleop") {
        
        // --- 1. SENSOR & SERVO INTERFACE ---
        // Subscribe to the 3-axis force sensor (Wrench topic)
        force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force_torque_sensor", 10,
            std::bind(&HeleneForceServoTeleop::force_callback, this, std::placeholders::_1));
        
        // Publisher targeting MoveIt Servo input topic
        servo_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);
        
        // Debug publisher for the filtered force
        filtered_force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/helene/teleop_filtered_force_z", 10);

        // --- 2. PARAMETER CONFIGURATION ---
        // Admittance gains: maps applied force (N) to Cartesian velocity (m/s)
        this->declare_parameter<double>("gain_x", 0.002);
        this->declare_parameter<double>("gain_y", 0.002);
        this->declare_parameter<double>("gain_z", 0.003);
        // Deadzone threshold in Newtons to suppress sensor drift/noise
        this->declare_parameter<double>("deadzone", 1.5); 
        // Target frame for the directional commands ("base_link" or "axis_6")
        this->declare_parameter<std::string>("command_frame", "base_link"); 

        // Filter and calibration setup (IIR filter matching your baseline task)
        bias_acquired_ = false;
        bias_count_ = 0;
        x_bias_ = 0.0; y_bias_ = 0.0; z_bias_ = 0.0;
        first_msg_ = true;
        a1_ = 0.993; b0_ = 0.668; b1_ = -0.662; 

        RCLCPP_INFO(this->get_logger(), "Force Teleop Node started. Calibrating sensor bias...");
    }

private:
    void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        double fx_raw = msg->wrench.force.x;
        double fy_raw = msg->wrench.force.y;
        double fz_raw = msg->wrench.force.z;

        // --- 1. SENSOR BIAS CALIBRATION ---
        // Collect first 100 samples to average out initial weight and offset
        if (!bias_acquired_) {
            x_bias_ += fx_raw;
            y_bias_ += fy_raw;
            z_bias_ += fz_raw;
            bias_count_++;
            if (bias_count_ >= 100) {
                x_bias_ /= 100.0;
                y_bias_ /= 100.0;
                z_bias_ /= 100.0;
                bias_acquired_ = true;
                RCLCPP_INFO(this->get_logger(), "Bias calibrated. Sensor ready. You can now guide the robot.");
            }
            return;
        }

        // Apply bias compensation
        double fx_meas = fx_raw - x_bias_;
        double fy_meas = fy_raw - y_bias_;
        double fz_meas = fz_raw - z_bias_;

        if (first_msg_) {
            z_meas_prev_ = fz_meas; z_comp_prev_ = fz_meas;
            first_msg_ = false;
        }

        // --- 2. SIGNAL FILTERING (IIR Filter applied to Z axis) ---
        double z_comp = (a1_ * z_comp_prev_) + (b0_ * fz_meas) + (b1_ * z_meas_prev_);
        z_meas_prev_ = fz_meas; z_comp_prev_ = z_comp;

        // Publish filtered Z force data for evaluation in PlotJuggler/RViz
        std_msgs::msg::Float64 filtered_msg;
        filtered_msg.data = z_comp;
        filtered_force_pub_->publish(filtered_msg);

        // --- 3. ADMITTANCE AND DEADBAND LOGIC ---
        double deadzone = this->get_parameter("deadzone").as_double();
        double gain_x = this->get_parameter("gain_x").as_double();
        double gain_y = this->get_parameter("gain_y").as_double();
        double gain_z = this->get_parameter("gain_z").as_double();
        std::string command_frame = this->get_parameter("command_frame").as_string();

        auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = this->now();
        twist_msg->header.frame_id = command_frame; 

        // If force exceeds the deadzone, calculate velocity; otherwise, stop movement
        twist_msg->twist.linear.x = (std::abs(fx_meas) > deadzone) ? fx_meas * gain_x : 0.0;
        twist_msg->twist.linear.y = (std::abs(fy_meas) > deadzone) ? fy_meas * gain_y : 0.0;
        twist_msg->twist.linear.z = (std::abs(z_comp)  > deadzone) ? z_comp  * gain_z : 0.0;

        // Keep orientation locked (no angular velocities generated for now)
        twist_msg->twist.angular.x = 0.0;
        twist_msg->twist.angular.y = 0.0;
        twist_msg->twist.angular.z = 0.0;

        // --- 4. DISPATCH COMMAND TO MOVEIT SERVO ---
        servo_pub_->publish(*twist_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_force_pub_;

    bool bias_acquired_;
    int bias_count_;
    double x_bias_, y_bias_, z_bias_;
    bool first_msg_;
    double z_meas_prev_, z_comp_prev_, a1_, b0_, b1_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeleneForceServoTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}