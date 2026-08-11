#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class HeleneForceServoTeleop : public rclcpp::Node {
public:
    HeleneForceServoTeleop() : Node("helene_force_servo_teleop") {

        // --- 1. PARAMETER DECLARATION ---
        // Force-to-velocity gains
        this->declare_parameter<double>("gain_x", 4.0);
        this->declare_parameter<double>("gain_y", 4.0);
        this->declare_parameter<double>("gain_z", 4.0);

        // Soft Deadband threshold in Newtons
        this->declare_parameter<double>("deadzone", 0.3);

        // Absolute Cartesian velocity limit in m/s
        this->declare_parameter<double>("max_linear_vel", 0.08);

        // Reference frame setup
        this->declare_parameter<std::string>("sensor_frame", "axis_6");
        this->declare_parameter<std::string>("command_frame", "base_link");

        // Electronic bias offsets of the sensor (measured with zero payload)
        this->declare_parameter<double>("elec_bias_x", 0.0);
        this->declare_parameter<double>("elec_bias_y", 0.0);
        this->declare_parameter<double>("elec_bias_z", 0.0);

        // Payload/Tool mass attached downstream of the sensor in kg
        this->declare_parameter<double>("payload_mass", 0.0);

        // EMA Low-Pass Filter Alpha coefficient (0.0 < alpha <= 1.0)
        this->declare_parameter<double>("ema_alpha", 0.25);

        // --- 2. INTERFACE INITIALIZATION ---
        force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force_torque_sensor", 10,
            std::bind(&HeleneForceServoTeleop::force_callback, this, std::placeholders::_1));

        servo_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);

        filtered_force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/helene/teleop_filtered_force_z", 10);

        // --- 3. TF2 LISTENER SETUP ---
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // --- 4. STATE VARIABLES INITIALIZATION ---
        first_msg_ = true;
        x_comp_prev_ = 0.0;
        y_comp_prev_ = 0.0;
        z_comp_prev_ = 0.0;

        RCLCPP_INFO(this->get_logger(),
            "Helene Force Servo Teleop Node started with Dynamic Gravity Compensation.");
    }

private:
    /**
     * @brief Applies a soft deadband threshold to prevent sudden velocity jumps upon crossing the deadzone.
     * @param force Input force value in Newtons.
     * @param dz Deadzone threshold in Newtons.
     * @return Effective force value after deadband subtraction.
     */
    double apply_soft_deadband(double force, double dz) {
        if (std::abs(force) <= dz) {
            return 0.0;
        }
        return (force > 0.0) ? (force - dz) : (force + dz);
    }

    void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        // Read configuration parameters
        double elec_bias_x = this->get_parameter("elec_bias_x").as_double();
        double elec_bias_y = this->get_parameter("elec_bias_y").as_double();
        double elec_bias_z = this->get_parameter("elec_bias_z").as_double();
        double payload_mass = this->get_parameter("payload_mass").as_double();
        double alpha = this->get_parameter("ema_alpha").as_double();

        std::string sensor_frame = this->get_parameter("sensor_frame").as_string();
        std::string command_frame = this->get_parameter("command_frame").as_string();

        // 1. SUBTRACT ELECTRONIC SENSOR BIAS (IN SENSOR FRAME)
        double fx_raw = msg->wrench.force.x - elec_bias_x;
        double fy_raw = msg->wrench.force.y - elec_bias_y;
        double fz_raw = msg->wrench.force.z - elec_bias_z;

        // 2. EXPONENTIAL MOVING AVERAGE (EMA) LOW-PASS FILTERING
        if (first_msg_) {
            x_comp_prev_ = fx_raw;
            y_comp_prev_ = fy_raw;
            z_comp_prev_ = fz_raw;
            first_msg_ = false;
        }

        double x_filt = alpha * fx_raw + (1.0 - alpha) * x_comp_prev_;
        double y_filt = alpha * fy_raw + (1.0 - alpha) * y_comp_prev_;
        double z_filt = alpha * fz_raw + (1.0 - alpha) * z_comp_prev_;

        x_comp_prev_ = x_filt;
        y_comp_prev_ = y_filt;
        z_comp_prev_ = z_filt;

        // 3. TRANSFORM FORCE VECTOR FROM SENSOR FRAME TO BASE FRAME
        geometry_msgs::msg::Vector3Stamped f_in, f_base;
        f_in.header.frame_id = sensor_frame;
        f_in.header.stamp = msg->header.stamp;
        f_in.vector.x = x_filt;
        f_in.vector.y = y_filt;
        f_in.vector.z = z_filt;

        try {
            geometry_msgs::msg::TransformStamped tf =
                tf_buffer_->lookupTransform(command_frame, sensor_frame, tf2::TimePointZero);
            tf2::doTransform(f_in, f_base, tf);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF lookup %s -> %s failed: %s", sensor_frame.c_str(), command_frame.c_str(), ex.what());
            return;
        }

        // 4. DYNAMIC GRAVITY COMPENSATION (IN BASE FRAME)
        // Gravity always points along -Z in base_link.
        // Adding (+ mass * g) removes tool weight regardless of current wrist orientation.
        constexpr double g = 9.81;
        double f_ext_x = f_base.vector.x;
        double f_ext_y = f_base.vector.y;
        double f_ext_z = f_base.vector.z + (payload_mass * g);

        // Publish net external Z force for diagnostic inspection (e.g. rqt_plot)
        std_msgs::msg::Float64 fz_msg;
        fz_msg.data = f_ext_z;
        filtered_force_pub_->publish(fz_msg);

        // 5. DEADBAND AND VELOCITY COMPUTATION
        double deadzone = this->get_parameter("deadzone").as_double();
        double gain_x = this->get_parameter("gain_x").as_double();
        double gain_y = this->get_parameter("gain_y").as_double();
        double gain_z = this->get_parameter("gain_z").as_double();
        double max_linear_vel = this->get_parameter("max_linear_vel").as_double();

        double eff_fx = apply_soft_deadband(f_ext_x, deadzone);
        double eff_fy = apply_soft_deadband(f_ext_y, deadzone);
        double eff_fz = apply_soft_deadband(f_ext_z, deadzone);

        // Diagnostic log output throttled every 500ms
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Net Force in '%s' [N] -> X: %.4f | Y: %.4f | Z: %.4f",
            command_frame.c_str(), f_ext_x, f_ext_y, f_ext_z);

        // 6. PUBLISH COMMAND TWIST FOR MOVEIT SERVO
        auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = this->now();
        twist_msg->header.frame_id = command_frame;

        twist_msg->twist.linear.x = std::clamp(eff_fx * gain_x, -max_linear_vel, max_linear_vel);
        twist_msg->twist.linear.y = std::clamp(eff_fy * gain_y, -max_linear_vel, max_linear_vel);
        twist_msg->twist.linear.z = std::clamp(eff_fz * gain_z, -max_linear_vel, max_linear_vel);

        twist_msg->twist.angular.x = 0.0;
        twist_msg->twist.angular.y = 0.0;
        twist_msg->twist.angular.z = 0.0;

        servo_pub_->publish(*twist_msg);
    }

    // --- MEMBER VARIABLES ---
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_force_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool first_msg_;
    double x_comp_prev_;
    double y_comp_prev_;
    double z_comp_prev_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeleneForceServoTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}