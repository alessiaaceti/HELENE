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

// ARCHITECTURE (v2 - mass-damper admittance):
//
//  force_callback() [event-driven, irregular dt]
//      tare -> scale -> EMA filter (noise) -> TF rotation (axis_6 -> base_link)
//      -> saves only a force "target" (fx_cmd_, fy_cmd_, fz_cmd_)
//
//  control_timer_callback() [fixed frequency, constant dt]
//      deadband -> dynamic integration v_dot = (F/B - v)/tau -> clamp
//      -> publishes TwistStamped to moveit_servo
//
// The separation is intentional: time integration (the "inertia" effect)
// is numerically stable only with a constant dt. Doing it inside the
// sensor callback (irregular dt) is the most common cause of
// oscillations/instability in this type of controller.

class HeleneForceServoTeleop : public rclcpp::Node {
public:
    HeleneForceServoTeleop() : Node("helene_force_servo_teleop") {

        // --- 1. SENSOR AND SERVO INTERFACE ---
        force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force_torque_sensor", 10,
            std::bind(&HeleneForceServoTeleop::force_callback, this, std::placeholders::_1));

        servo_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);

        // Diagnostics: filtered Z force (post-EMA, pre-dynamics) and commanded
        // Z velocity (post-integration) - useful in rqt_plot/PlotJuggler
        // to tune deadzone/bias and damping/tau respectively.
        filtered_force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/helene/teleop_filtered_force_z", 10);
        velocity_diag_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/helene/teleop_commanded_velocity_z", 10);

        // --- 2. PARAMETER CONFIGURATION (TO BE CALIBRATED IN LAB) ---

        // Deadzone in Newtons. Tune it by observing the diagnostic log at rest,
        // without touching the sensor: set the value just above the
        // residual noise observed there.
        this->declare_parameter<double>("deadzone", 0.018); // Newtons

        // raw_meas_to_wrench.cpp ALREADY converts raw counts to Newtons
        // before publishing to /force_torque_sensor: scale_factor normally
        // remains 1.0 here, unless a deliberate calibrated correction is needed.
        this->declare_parameter<double>("scale_factor", 1.0);

        // --- Mass-damper dynamics parameters ---
        // Physical equivalent: M*v_dot + B*v = F_ext, with M = damping*tau.
        // damping_i ("sensitivity"): STEADY-STATE velocity for a given
        //   constant force -> v_steady = F / damping_i. Lower damping
        //   = higher sensitivity (a small push produces more velocity).
        // time_constant_i ("tau", in seconds): time required to reach
        //   steady state, and how much it "coasts" after release.
        //   Higher = smoother/heavier; lower = more responsive.
        //   Start from 0.15-0.35 s and tune by feel.
        this->declare_parameter<double>("damping_x", 40.0);
        this->declare_parameter<double>("damping_y", 40.0);
        this->declare_parameter<double>("damping_z", 40.0);
        this->declare_parameter<double>("time_constant_x", 0.25); // s
        this->declare_parameter<double>("time_constant_y", 0.25); // s
        this->declare_parameter<double>("time_constant_z", 0.25); // s

        // Integration/publishing loop frequency (Hz). Match it
        // with the expected frequency of moveit_servo in
        // servo_params.yaml. NOTE: changing it at runtime has no effect,
        // as the timer is created only once when starting the node.
        this->declare_parameter<double>("control_rate", 100.0); // Hz

        // If no valid force measurement arrives within this time,
        // the target is reset to zero for safety: the arm
        // decelerates smoothly (thanks to damping) instead of continuing
        // to move on stale commands.
        this->declare_parameter<double>("force_timeout", 0.3); // s

        // Below this threshold, velocity is forced strictly to
        // zero: pure exponential decay never reaches "true" zero,
        // and without this cutoff the controller would stay active
        // indefinitely at negligible velocity.
        this->declare_parameter<double>("velocity_epsilon", 0.0005); // m/s

        // Absolute safety limit on Cartesian velocity. Keep it
        // low until you verify behavior with the arm well
        // away from people/obstacles. Unit: m/s (ROS convention).
        this->declare_parameter<double>("max_linear_vel", 0.08); // m/s

        this->declare_parameter<std::string>("sensor_frame", "axis_6");
        this->declare_parameter<std::string>("command_frame", "base_link");

        // --- 3. STATE INITIALIZATION ---
        bias_acquired_ = false;
        bias_count_ = 0;
        x_bias_ = 0.0; y_bias_ = 0.0; z_bias_ = 0.0;
        first_msg_ = true;

        x_comp_prev_ = 0.0;
        y_comp_prev_ = 0.0;
        z_comp_prev_ = 0.0;

        fx_cmd_ = 0.0; fy_cmd_ = 0.0; fz_cmd_ = 0.0;
        vx_ = 0.0; vy_ = 0.0; vz_ = 0.0;
        last_force_stamp_ = this->now();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        double control_rate = this->get_parameter("control_rate").as_double();
        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / control_rate),
            std::bind(&HeleneForceServoTeleop::control_timer_callback, this));

        RCLCPP_INFO(this->get_logger(),
            "Force Teleop Node (mass-damper admittance) started. "
            "Tare calibration in progress: DO NOT TOUCH the sensor for ~2 seconds...");
    }

private:
    // Soft Deadband: avoids sudden jumps when exceeding the threshold
    double apply_soft_deadband(double force, double dz) {
        if (std::abs(force) <= dz) {
            return 0.0;
        }
        return (force > 0.0) ? (force - dz) : (force + dz);
    }

    // --- CALLBACK 1: event-driven (irregular dt) ---
    // Tare, scale, filter noise, rotate to command frame.
    // DOES NOT perform time integration: saves only the latest force target.
    void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        double fx_raw = msg->wrench.force.x;
        double fy_raw = msg->wrench.force.y;
        double fz_raw = msg->wrench.force.z;

        // 1. INITIAL TARE (DO NOT TOUCH THE SENSOR IN THIS PHASE)
        if (!bias_acquired_) {
            x_bias_ += fx_raw; y_bias_ += fy_raw; z_bias_ += fz_raw;
            bias_count_++;
            if (bias_count_ >= 100) {
                x_bias_ /= 100.0; y_bias_ /= 100.0; z_bias_ /= 100.0;
                bias_acquired_ = true;
                RCLCPP_INFO(this->get_logger(), "--- TARE COMPLETED. SENSOR READY ---");
            }
            return;
        }

        // 2. FORCE SCALING (typically scale_factor = 1.0, see comment above)
        double scale_factor = this->get_parameter("scale_factor").as_double();
        if (scale_factor == 0.0) scale_factor = 1.0;

        double fx_meas = -(fx_raw - x_bias_) / scale_factor;
        double fy_meas = -(fy_raw - y_bias_) / scale_factor;
        double fz_meas = (fz_raw - z_bias_) / scale_factor;

        if (first_msg_) {
            x_comp_prev_ = fx_meas;
            y_comp_prev_ = fy_meas;
            z_comp_prev_ = fz_meas;
            first_msg_ = false;
        }

        // --- 3. NOISE FILTERING (EMA) ---
        // This filter ONLY serves to clean sensor noise.
        // Perceived movement "smoothness" is controlled separately
        // by time_constant_i in the control timer.
        double alpha = 0.25;
        double x_comp = alpha * fx_meas + (1.0 - alpha) * x_comp_prev_;
        double y_comp = alpha * fy_meas + (1.0 - alpha) * y_comp_prev_;
        double z_comp = alpha * fz_meas + (1.0 - alpha) * z_comp_prev_;

        x_comp_prev_ = x_comp;
        y_comp_prev_ = y_comp;
        z_comp_prev_ = z_comp;

        std_msgs::msg::Float64 fz_msg;
        fz_msg.data = z_comp;
        filtered_force_pub_->publish(fz_msg);

        // --- 4. TF TRANSFORM TO COMMAND FRAME (fixed) ---
        std::string sensor_frame = this->get_parameter("sensor_frame").as_string();
        std::string command_frame = this->get_parameter("command_frame").as_string();

        geometry_msgs::msg::Vector3Stamped f_in, f_out;
        f_in.header.frame_id = sensor_frame;
        f_in.header.stamp = msg->header.stamp;
        f_in.vector.x = x_comp;
        f_in.vector.y = y_comp;
        f_in.vector.z = z_comp;

        try {
            geometry_msgs::msg::TransformStamped tf =
                tf_buffer_->lookupTransform(command_frame, sensor_frame, tf2::TimePointZero);
            tf2::doTransform(f_in, f_out, tf);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF %s -> %s not available: %s", sensor_frame.c_str(), command_frame.c_str(), ex.what());
            return; // target not updated: watchdog in timer will detect this
        }

        // Save target only: integration happens inside the fixed dt timer
        fx_cmd_ = f_out.vector.x;
        fy_cmd_ = f_out.vector.y;
        fz_cmd_ = f_out.vector.z;
        last_force_stamp_ = this->now();
    }

    // --- CALLBACK 2: fixed frequency (constant dt) ---
    // Mass-damper dynamics reside here: this is the only place that integrates
    // over time, and thus the single source of the feeling of "inertia".
    void control_timer_callback() {
        if (!bias_acquired_) return;

        double control_rate = this->get_parameter("control_rate").as_double();
        double dt = 1.0 / control_rate;

        // Watchdog: no fresh force data -> zero target, safe deceleration
        double force_timeout = this->get_parameter("force_timeout").as_double();
        double age = (this->now() - last_force_stamp_).seconds();
        double fx_target = fx_cmd_, fy_target = fy_cmd_, fz_target = fz_cmd_;
        if (age > force_timeout) {
            fx_target = 0.0; fy_target = 0.0; fz_target = 0.0;
        }

        double deadzone = this->get_parameter("deadzone").as_double();
        double eff_fx = apply_soft_deadband(fx_target, deadzone);
        double eff_fy = apply_soft_deadband(fy_target, deadzone);
        double eff_fz = apply_soft_deadband(fz_target, deadzone);

        double damping_x = this->get_parameter("damping_x").as_double();
        double damping_y = this->get_parameter("damping_y").as_double();
        double damping_z = this->get_parameter("damping_z").as_double();
        double tau_x = std::max(this->get_parameter("time_constant_x").as_double(), 1e-3);
        double tau_y = std::max(this->get_parameter("time_constant_y").as_double(), 1e-3);
        double tau_z = std::max(this->get_parameter("time_constant_z").as_double(), 1e-3);
        double max_linear_vel = this->get_parameter("max_linear_vel").as_double();
        double vel_eps = this->get_parameter("velocity_epsilon").as_double();

        if (damping_x == 0.0) damping_x = 1.0;
        if (damping_y == 0.0) damping_y = 1.0;
        if (damping_z == 0.0) damping_z = 1.0;

        // Dynamics: v_dot = (F/damping - v) / tau
        // (equivalent to M*v_dot + B*v = F with M = damping*tau)
        vx_ += dt * ((eff_fx / damping_x) - vx_) / tau_x;
        vy_ += dt * ((eff_fy / damping_y) - vy_) / tau_y;
        vz_ += dt * ((eff_fz / damping_z) - vz_) / tau_z;

        vx_ = std::clamp(vx_, -max_linear_vel, max_linear_vel);
        vy_ = std::clamp(vy_, -max_linear_vel, max_linear_vel);
        vz_ = std::clamp(vz_, -max_linear_vel, max_linear_vel);

        if (std::abs(vx_) < vel_eps) vx_ = 0.0;
        if (std::abs(vy_) < vel_eps) vy_ = 0.0;
        if (std::abs(vz_) < vel_eps) vz_ = 0.0;

        std::string command_frame = this->get_parameter("command_frame").as_string();

        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = command_frame;
        twist_msg.twist.linear.x = vx_;
        twist_msg.twist.linear.y = vy_;
        twist_msg.twist.linear.z = vz_;
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;
        servo_pub_->publish(twist_msg);

        std_msgs::msg::Float64 vz_diag;
        vz_diag.data = vz_;
        velocity_diag_pub_->publish(vz_diag);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Target F [%s, N] -> X:%.4f Y:%.4f Z:%.4f | Commanded v [m/s] -> X:%.4f Y:%.4f Z:%.4f",
            command_frame.c_str(), eff_fx, eff_fy, eff_fz, vx_, vy_, vz_);
    }

    // --- MEMBER VARIABLES ---
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_force_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_diag_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool bias_acquired_;
    int bias_count_;
    double x_bias_, y_bias_, z_bias_;
    bool first_msg_;

    double x_comp_prev_;
    double y_comp_prev_;
    double z_comp_prev_;

    // Force target (updated by the sensor callback)
    double fx_cmd_, fy_cmd_, fz_cmd_;
    // Integrated velocity state (persistent across timer ticks:
    // literally the system's "memory/inertia")
    double vx_, vy_, vz_;
    rclcpp::Time last_force_stamp_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeleneForceServoTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}