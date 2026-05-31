#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// KDL Libraries
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

using namespace std::chrono_literals;

enum class TaskState {
    APPROACH,
    SEARCH,
    INSERTION
};

class ForceFeedbackNode : public rclcpp::Node {
public:
    ForceFeedbackNode() : Node("force_feedback_node"), kdl_ready_(false) {
        
        // --- 1. Sensor Subscriptions ---
        force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force_torque_sensor", 10, // QoS: KeepLast(10) for real-time data
            std::bind(&ForceFeedbackNode::force_callback, this, std::placeholders::_1));
        
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ForceFeedbackNode::joint_state_callback, this, std::placeholders::_1));

        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/helene_velocity_controller/commands", 10);
        
        filtered_force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/helene/filtered_force_z", 10);

        // --- 2. URDF Acquisition to create the KDL Jacobian ---
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", qos,
            std::bind(&ForceFeedbackNode::urdf_callback, this, std::placeholders::_1));

        // Link names from URDF (verify that base_link and axis_6 are correct for your model)
        base_link_name_ = "base_link";
        tip_link_name_ = "axis_6";

        // Joint names for correct ordering
        joint_names_ = {"q1", "q2", "q3", "q4", "q5", "q6"};

        // --- State Machine & Filter Parameters ---
        current_state_ = TaskState::APPROACH;
        contact_threshold_ = 10.0; // Newtons, adjust based on expected contact force
        hole_drop_threshold_ = 3.0; // Newtons, adjust based on expected force drop when hole is found
        first_msg_ = true;
        z_meas_prev_ = 0.0; z_comp_prev_ = 0.0;
        a1_ = 0.993; b0_ = 0.668; b1_ = -0.662; 
        theta_ = 0.0; omega_ = 2.0; spiral_b_ = 0.01;
        admittance_gain_ = 0.01; deadzone_ = 1.0;       

        RCLCPP_INFO(this->get_logger(), "Node Started. Waiting for URDF file to build the Jacobian...");
    }

private:
    // --- CALLBACK 1: Reads the URDF and prepares the math ---
    void urdf_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (kdl_ready_) return;

        KDL::Tree tree;
        if (!kdl_parser::treeFromString(msg->data, tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL tree from URDF.");
            return;
        }

        if (!tree.getChain(base_link_name_, tip_link_name_, chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create chain from %s to %s.", base_link_name_.c_str(), tip_link_name_.c_str());
            return;
        }

        ik_solver_vel_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_);
        q_current_.resize(chain_.getNrOfJoints());
        
        kdl_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Jacobian initialized! Chain created with %d joints. Peg-in-Hole: APPROACH", chain_.getNrOfJoints());
    }

    // --- CALLBACK 2: Constantly updates the real motor positions ---
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!kdl_ready_) return;
        
        // Ensure that the joint order matches the KDL matrix order
        for (size_t i = 0; i < msg->name.size(); i++) {
            for (size_t j = 0; j < joint_names_.size(); j++) {
                if (msg->name[i] == joint_names_[j]) {
                    q_current_(j) = msg->position[i];
                }
            }
        }
    }

    // --- CALLBACK 3: Force Calculation, State Machine, and Inverse Jacobian ---
    void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        if (!kdl_ready_) return; // Wait for kinematics to be ready
        
        double fx = msg->wrench.force.x;
        double fy = msg->wrench.force.y;
        double z_raw = msg->wrench.force.z;
        
        // --- BIAS ACQUISITION ---
        if (!bias_acquired_) {
            z_bias_ += z_raw;
            bias_count_++;
            if (bias_count_ >= 100) {
                z_bias_ /= 100.0;
                bias_acquired_ = true;
                RCLCPP_INFO(this->get_logger(), "BIAS Acquired. Bias Z = %.2f", z_bias_);
            }
            return; // Don't proceed until bias is acquired
        }

        // --- 1. FORCE PROCESSING: Bias Removal + Creep Filter ---
        double z_meas = z_raw - z_bias_;

        if (first_msg_) {
            z_meas_prev_ = z_meas; 
            z_comp_prev_ = z_meas;
            last_time_ = this->now();           // Initialize time for the first message
            state_start_time_ = this->now();    // Initialize state start time
            first_msg_ = false;
        }

        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0 || dt > 0.1) {
            dt = 0.01; 
        }
        last_time_ = current_time;      

        // Creep (Kriechverhalten) Filter
        double z_comp = (a1_ * z_comp_prev_) + (b0_ * z_meas) + (b1_ * z_meas_prev_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Z_RAW: %.3f | Z_FILTERED: %.3f", z_meas, z_comp);
        z_meas_prev_ = z_meas; z_comp_prev_ = z_comp;

        // Publish the filtered force for monitoring
        std_msgs::msg::Float64 filtered_msg;
        filtered_msg.data = z_comp;
        filtered_force_pub_->publish(filtered_msg);

        double vx = 0.0, vy = 0.0, vz = 0.0;
        double duration = (this->now() - state_start_time_).seconds();

        // 2. Logic Machine (Cartesian Velocities)
        switch (current_state_) {
            case TaskState::APPROACH:
            {
                double nominal_vz = -0.0005; // Descent
                // Damping factor to slow down as we approach the contact point
                // The idea is to reduce descent speed as we get closer to the contact threshold, creating a "soft landing" effect
                double damping_factor = 0.0001; 
                vz = nominal_vz + (std::abs(z_comp) * damping_factor);
                
                // Safety check to prevent aggressive commands if we are above the contact threshold
                if (vz > 0) vz = 0.0;
                if (std::abs(z_meas) > contact_threshold_) {
                    RCLCPP_INFO(this->get_logger(), "Contact! (Fz=%.2f N). Starting SPIRAL.", z_meas);
                    vx = 0.0; vy = 0.0; vz = 0.0; // Stop descent immediately on contact
                    current_state_ = TaskState::SEARCH;
                    state_start_time_ = this->now();
                }
            }
            break;

            case TaskState::SEARCH:
                theta_ += omega_ * dt; 
                vx = spiral_b_ * omega_ * (std::cos(theta_) - theta_ * std::sin(theta_));
                vy = spiral_b_ * omega_ * (std::sin(theta_) + theta_ * std::cos(theta_));
                // As we spiral, we also want to maintain a gentle downward force to ensure we stay in contact with the surface
                vz = (std::abs(z_comp) - contact_threshold_) * 0.001;

                duration = (this->now() - state_start_time_).seconds();
                if (duration > 2.0 && std::abs(z_comp) < hole_drop_threshold_) {
                    RCLCPP_INFO(this->get_logger(), "hole found!");
                    current_state_ = TaskState::INSERTION;
                    state_start_time_ = this->now();
                }
                break;

            case TaskState::INSERTION:
                vz = -0.002; 
                if (std::abs(fx) > deadzone_) vx = fx * admittance_gain_;
                if (std::abs(fy) > deadzone_) vy = fy * admittance_gain_;
                break;
        }

        // --- 3. CORE OF THE THESIS: INVERSE JACOBIAN ---
        // Desired Cartesian velocity vector
        KDL::Twist v_cartesian;
        v_cartesian.vel.x(vx);
        v_cartesian.vel.y(vy);
        v_cartesian.vel.z(vz);
        v_cartesian.rot.x(0.0); // No rotation requested for the flange
        v_cartesian.rot.y(0.0);
        v_cartesian.rot.z(0.0);

        KDL::JntArray q_dot_out(chain_.getNrOfJoints());
        
        // Solver: \dot{q} = J^{-1}(q) * v
        int status = ik_solver_vel_->CartToJnt(q_current_, v_cartesian, q_dot_out);

        if (status < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "KDL Solver Error. Kinematic singularity?");
            return;
        }

        // --- 4. Publishing to Gazebo Controller ---
        auto velocities_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
        velocities_msg->data.resize(chain_.getNrOfJoints(), 0.0);

        for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
            // Limit the maximum velocity to prevent aggressive commands
            double v_cmd = q_dot_out(i);
            if (v_cmd > 0.2) v_cmd = 0.2;
            if (v_cmd < -0.2) v_cmd = -0.2;

            velocities_msg->data[i] = v_cmd;
        }
        velocity_pub_->publish(*velocities_msg);
    }

    // ROS and KDL Variables
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_force_pub_;
    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_vel_;
    KDL::JntArray q_current_;
    bool kdl_ready_;
    std::string base_link_name_, tip_link_name_;
    std::vector<std::string> joint_names_;

    // State Machine Variables
    rclcpp::Time state_start_time_;
    rclcpp::Time last_time_;
    double z_bias_ = 0.0;
    bool bias_acquired_ = false;
    int bias_count_ = 0;
    TaskState current_state_;
    double contact_threshold_, hole_drop_threshold_;
    bool first_msg_;
    double z_meas_prev_, z_comp_prev_, a1_, b0_, b1_;
    double theta_, omega_, spiral_b_;
    double admittance_gain_, deadzone_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForceFeedbackNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}