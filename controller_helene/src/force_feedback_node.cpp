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

// KDL (Kinematics and Dynamics Library) for Jacobian calculation
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

using namespace std::chrono_literals;

/**
 * State machine stages for the Peg-in-Hole task
 */
enum class TaskState {
    APPROACH,  // Moving down until contact with the surface
    SEARCH,    // Executing spiral motion to locate the hole
    INSERTION  // Descending into the hole with admittance control
};

class ForceFeedbackNode : public rclcpp::Node {
public:
    ForceFeedbackNode() : Node("force_feedback_node"), kdl_ready_(false) {
        
        // --- 1. SENSOR & ACTUATOR INTERFACE ---
        // Subscribe to Force/Torque sensor data (Wrench)
        force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force_torque_sensor", 10,
            std::bind(&ForceFeedbackNode::force_callback, this, std::placeholders::_1));
        
        // Subscribe to current joint positions for Kinematics
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ForceFeedbackNode::joint_state_callback, this, std::placeholders::_1));

        // Command publisher for joint velocities (Gazebo or Hardware controllers)
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/helene_velocity_controller/commands", 10);
        
        // Debug publisher for the filtered Z-force
        filtered_force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/helene/filtered_force_z", 10);

        // --- 2. ROBOT MODEL ACQUISITION ---
        // Retrieve URDF from /robot_description to build the KDL tree
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", qos,
            std::bind(&ForceFeedbackNode::urdf_callback, this, std::placeholders::_1));

        base_link_name_ = "base_link";
        tip_link_name_ = "axis_6";
        joint_names_ = {"q1", "q2", "q3", "q4", "q5", "q6"};

        // --- 3. PARAMETER INITIALIZATION ---
        current_state_ = TaskState::APPROACH;
        contact_threshold_ = 10.0;     // Force in N to detect surface
        hole_drop_threshold_ = 3.0;    // Force drop in N to detect hole entrance
        first_msg_ = true;

        // Filter coefficients (Infinite Impulse Response - IIR Filter)
        a1_ = 0.993; b0_ = 0.668; b1_ = -0.662; 
        
        // Spiral Search parameters: r = spiral_b * theta
        theta_ = 0.0; 
        omega_ = 2.0;      // Angular velocity
        spiral_b_ = 0.01;  // Spiral growth rate
        
        // Admittance control: v = gain * Force
        admittance_gain_ = 0.01; 
        deadzone_ = 1.0;       

        RCLCPP_INFO(this->get_logger(), "Node Started. Waiting for URDF...");
    }

private:
    /**
     * Parses the URDF to initialize the KDL Chain and the Velocity IK Solver
     */
    void urdf_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (kdl_ready_) return;

        KDL::Tree tree;
        if (!kdl_parser::treeFromString(msg->data, tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL tree.");
            return;
        }

        if (!tree.getChain(base_link_name_, tip_link_name_, chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create KDL chain.");
            return;
        }

        // Initialize Pseudo-Inverse solver for differential kinematics
        ik_solver_vel_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_);
        q_current_.resize(chain_.getNrOfJoints());
        
        kdl_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "KDL Ready. Starting Task: APPROACH");
    }

    /**
     * Updates the internal joint array ensuring correct joint mapping
     */
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!kdl_ready_) return;
        
        for (size_t i = 0; i < msg->name.size(); i++) {
            for (size_t j = 0; j < joint_names_.size(); j++) {
                if (msg->name[i] == joint_names_[j]) {
                    q_current_(j) = msg->position[i];
                }
            }
        }
    }

    /**
     * Main Control Loop: Force Processing -> State Machine -> IK -> Command
     */
    void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        if (!kdl_ready_) return; 
        
        double fx = msg->wrench.force.x;
        double fy = msg->wrench.force.y;
        double z_raw = msg->wrench.force.z;
        
        // --- 1. SENSOR BIAS CALIBRATION ---
        // Collect first 100 samples to average out the weight of the tool/sensor offset
        if (!bias_acquired_) {
            z_bias_ += z_raw;
            bias_count_++;
            if (bias_count_ >= 100) {
                z_bias_ /= 100.0;
                bias_acquired_ = true;
                RCLCPP_INFO(this->get_logger(), "Bias calibrated: %.2f", z_bias_);
            }
            return;
        }

        // --- 2. SIGNAL FILTERING ---
        double z_meas = z_raw - z_bias_;
        if (first_msg_) {
            z_meas_prev_ = z_meas; z_comp_prev_ = z_meas;
            last_time_ = this->now(); state_start_time_ = this->now();
            first_msg_ = false;
        }

        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0 || dt > 0.1) dt = 0.01;
        last_time_ = current_time;      

        // Apply IIR Filter for creep compensation
        double z_comp = (a1_ * z_comp_prev_) + (b0_ * z_meas) + (b1_ * z_meas_prev_);
        z_meas_prev_ = z_meas; z_comp_prev_ = z_comp;

        std_msgs::msg::Float64 filtered_msg;
        filtered_msg.data = z_comp;
        filtered_force_pub_->publish(filtered_msg);

        // Cartesian velocities to compute
        double vx = 0.0, vy = 0.0, vz = 0.0;
        double duration = (this->now() - state_start_time_).seconds();

        // --- 3. TASK LOGIC (STATE MACHINE) ---
        switch (current_state_) {
            case TaskState::APPROACH:
                // Descent with soft-landing: speed decreases as force increases
                vz = -0.0005 + (std::abs(z_comp) * 0.00001);
                if (vz > 0) vz = 0.0;

                // Contact Detection
                if (std::abs(z_meas) > contact_threshold_) {
                    RCLCPP_INFO(this->get_logger(), "Contact! (Fz=%.2f N). RELEASING PRESSURE.", z_meas);
                    current_state_ = TaskState::SEARCH;
                    state_start_time_ = this->now();
                }
                break;

            case TaskState::SEARCH:
                // A. Generate Spiral Velocities (X-Y)
                theta_ += omega_ * dt;
                vx = std::clamp(spiral_b_ * omega_ * (std::cos(theta_) - theta_ * std::sin(theta_)), -0.003, 0.003);
                vy = std::clamp(spiral_b_ * omega_ * (std::sin(theta_) + theta_ * std::cos(theta_)), -0.003, 0.003);
                
                // B. Maintain constant contact force in Z (Proportional Control)
                vz = -((contact_threshold_ - z_comp) * 0.00005);
                vz = std::clamp(vz, -0.002, 0.002);
                
                // C. Hole Detection: If force drops significantly, we are above the hole
                if (duration > 1.5 && z_comp < hole_drop_threshold_) {
                    RCLCPP_INFO(this->get_logger(), "Hole located! Starting INSERTION.");
                    current_state_ = TaskState::INSERTION;
                    state_start_time_ = this->now();
                }
                break;

            case TaskState::INSERTION:
                // Controlled descent
                vz = -0.002; 
                // Admittance control: if tool hits the hole sides, move in the direction of force
                if (std::abs(fx) > deadzone_) vx = fx * admittance_gain_;
                if (std::abs(fy) > deadzone_) vy = fy * admittance_gain_;
                break;
        }

        // --- 4. DIFFERENTIAL KINEMATICS ---
        KDL::Twist v_cartesian;
        v_cartesian.vel.x(vx); v_cartesian.vel.y(vy); v_cartesian.vel.z(vz);
        v_cartesian.rot.x(0.0); v_cartesian.rot.y(0.0); v_cartesian.rot.z(0.0);

        KDL::JntArray q_dot_out(chain_.getNrOfJoints());
        
        // Solve IK: Map Cartesian Twist to Joint Velocities
        if (ik_solver_vel_->CartToJnt(q_current_, v_cartesian, q_dot_out) < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "IK Solver Fail.");
            return;
        }

        // --- 5. PUBLISH COMMANDS ---
        auto velocities_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
        velocities_msg->data.resize(chain_.getNrOfJoints());

        for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
            // Safety saturation: Limit max joint speed to 0.2 rad/s
            velocities_msg->data[i] = std::clamp(q_dot_out(i), -0.2, 0.2);
        }
        velocity_pub_->publish(*velocities_msg);
    }

    // Member variables for ROS, KDL and State
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

    rclcpp::Time state_start_time_, last_time_;
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