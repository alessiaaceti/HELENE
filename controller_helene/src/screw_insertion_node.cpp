// Force-guided screw insertion: approach, spiral search, hole detection, insertion.
// The algorithm lives in controller_helene/insertion_controller.hpp (ROS independent).
// This node feeds it the force (tared, rotated into base_link) and the tool position
// and forwards the returned Cartesian velocity to MoveIt Servo.
#include <chrono>
#include <deque>
#include <cmath>
#include <limits>
#include <memory>
#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "controller_helene/insertion_controller.hpp"

#include <Eigen/Dense>
#include "kdl/chain.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace controller_helene;

class ScrewInsertionNode : public rclcpp::Node
{
public:
  ScrewInsertionNode() : Node("screw_insertion_node")
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    InsertionParams d;
    auto dbl = [&](const std::string & n, double v) { return declare_parameter<double>(n, v); };
    auto integer = [&](const std::string & n, int v) { return static_cast<int>(declare_parameter<int>(n, v)); };

    InsertionParams p;
    p.tare_samples = integer("tare_samples", d.tare_samples);
    p.filter_cutoff_hz = dbl("filter_cutoff_hz", d.filter_cutoff_hz);
    p.approach_speed = dbl("approach_speed", d.approach_speed);
    const double ez = dbl("expected_surface_z", nan);
    if (!std::isnan(ez)) p.expected_surface_z = ez;
    p.contact_force = dbl("contact_force", d.contact_force);
    p.contact_samples = integer("contact_samples", d.contact_samples);
    p.target_force = dbl("target_force", d.target_force);
    p.force_gain = dbl("force_gain", d.force_gain);
    p.force_deadband = dbl("force_deadband", d.force_deadband);
    p.max_z_speed = dbl("max_z_speed", d.max_z_speed);
    p.spiral_pitch = dbl("spiral_pitch", d.spiral_pitch);
    p.spiral_speed = dbl("spiral_speed", d.spiral_speed);
    p.spiral_min_radius = dbl("spiral_min_radius", d.spiral_min_radius);
    p.spiral_max_radius = dbl("spiral_max_radius", d.spiral_max_radius);
    p.xy_gain = dbl("xy_gain", d.xy_gain);
    p.hole_force_ratio = dbl("hole_force_ratio", d.hole_force_ratio);
    p.hole_depth = dbl("hole_depth", d.hole_depth);
    p.hole_samples = integer("hole_samples", d.hole_samples);
    p.lost_force_ratio = dbl("lost_force_ratio", d.lost_force_ratio);
    p.insert_speed = dbl("insert_speed", d.insert_speed);
    p.insert_force = dbl("insert_force", d.insert_force);
    p.insert_xy_gain = dbl("insert_xy_gain", d.insert_xy_gain);
    p.insert_depth = dbl("insert_depth", d.insert_depth);
    p.min_insert_depth = dbl("min_insert_depth", d.min_insert_depth);
    p.insert_max_force = dbl("insert_max_force", d.insert_max_force);
    p.stall_time = dbl("stall_time", d.stall_time);
    p.insert_timeout = dbl("insert_timeout", d.insert_timeout);
    p.abort_force = dbl("abort_force", d.abort_force);
    p.max_lateral_force = dbl("max_lateral_force", d.max_lateral_force);
    p.rim_depth = dbl("rim_depth", d.rim_depth);
    params_ = p;

    // Sign of each sensor axis so that, in the sensor frame, +z is the reaction of the surface
    // on the tool pointing away from the surface. The hand-guiding node uses (-1, -1, +1).
    sign_ = {dbl("sensor_sign_x", -1.0), dbl("sensor_sign_y", -1.0), dbl("sensor_sign_z", 1.0)};
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "axis_6");
    command_frame_ = declare_parameter<std::string>("command_frame", "base_link");
    tool_frame_ = declare_parameter<std::string>("tool_frame", "axis_6");
    tool_tip_z_ = dbl("tool_tip_offset_z", 0.0);   // m, screw tip along tool +z
    rate_ = dbl("control_rate", 100.0);
    force_timeout_ = dbl("force_timeout", 0.3);
    auto_start_ = declare_parameter<bool>("auto_start", true);
    max_speed_ = dbl("max_speed", 0.01);
    sensor_delay_ = dbl("sensor_delay", 0.0);   // s, artificial extra delay of the force signal
    // direct_joint_control: map the Cartesian velocity to joint velocities here (damped
    // least squares on the KDL Jacobian) and publish them to the velocity controller. MoveIt
    // Servo has a dead band of about 1 mm/s that swallows the sub-mm/s speeds of a force loop.
    direct_ = declare_parameter<bool>("direct_joint_control", false);
    joint_cmd_topic_ = declare_parameter<std::string>("joint_command_topic", "/helene_velocity_controller/commands");
    dls_lambda_ = dbl("dls_lambda", 0.02);
    max_joint_vel_ = dbl("max_joint_velocity", 0.5);
    if (direct_) setupKinematics();

    force_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/force_torque_sensor", rclcpp::SensorDataQoS().keep_last(10),
      [this](geometry_msgs::msg::WrenchStamped::SharedPtr m) {
        // optional artificial dead time (simulation: CAN + micro-ROS + ROS latency of the real chain)
        queue_.push_back({now(), *m});
        while (queue_.size() > 1 && (now() - queue_[1].first).seconds() >= sensor_delay_) {
          queue_.pop_front();
        }
        if ((now() - queue_.front().first).seconds() >= sensor_delay_) {
          last_wrench_ = queue_.front().second;
          last_force_time_ = now();
          have_force_ = true;
        }
      });
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
    if (direct_) {
      joint_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(joint_cmd_topic_, 10);
      js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr m) {
          for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) q_[m->name[i]] = m->position[i];
        });
    }
    state_pub_ = create_publisher<std_msgs::msg::String>("/helene/insertion_state", 10);
    fz_pub_ = create_publisher<std_msgs::msg::Float64>("/helene/insertion_fz", 10);
    depth_pub_ = create_publisher<std_msgs::msg::Float64>("/helene/insertion_depth", 10);
    tip_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/helene/insertion_tip", 10);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        restart();
        res->success = true;
        res->message = "insertion restarted (tare first: keep the arm still)";
      });
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        running_ = false;
        res->success = true;
        res->message = "stopped";
      });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    ctrl_ = std::make_unique<ScrewInsertion>(params_);
    running_ = auto_start_;
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_), [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
      "Screw insertion node ready (auto_start=%s). Keep the arm still during the tare. "
      "Start/stop with ~/start and ~/stop.", auto_start_ ? "true" : "false");
  }

private:
  void restart()
  {
    ctrl_ = std::make_unique<ScrewInsertion>(params_);
    last_state_ = State::TARE;
    running_ = true;
  }

  void setupKinematics()
  {
    const std::string urdf = declare_parameter<std::string>("robot_description", "");
    KDL::Tree tree;
    if (urdf.empty() || !kdl_parser::treeFromString(urdf, tree) ||
        !tree.getChain(command_frame_, tool_frame_, chain_)) {
      RCLCPP_FATAL(get_logger(), "direct_joint_control needs a valid robot_description parameter");
      throw std::runtime_error("robot_description");
    }
    // the tool tip is a fixed offset along the tool z axis
    chain_.addSegment(KDL::Segment("tool_tip", KDL::Joint(KDL::Joint::None),
                                   KDL::Frame(KDL::Vector(0, 0, tool_tip_z_))));
    for (unsigned i = 0; i < chain_.getNrOfSegments(); ++i) {
      const auto & j = chain_.getSegment(i).getJoint();
      if (j.getType() != KDL::Joint::None) joint_names_.push_back(j.getName());
    }
    jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    // Tip position in the base frame comes from KDL as well (no tf delay, same model as the Jacobian)
    RCLCPP_INFO(get_logger(), "direct joint control over %zu joints", joint_names_.size());
  }

  // Cartesian linear velocity (zero angular velocity) -> joint velocities, damped least squares.
  bool cartesianToJoints(const Vec3 & v, std::vector<double> & qd)
  {
    const size_t n = joint_names_.size();
    KDL::JntArray q(n);
    for (size_t i = 0; i < n; ++i) {
      auto it = q_.find(joint_names_[i]);
      if (it == q_.end()) return false;
      q(i) = it->second;
    }
    KDL::Jacobian J(n);
    if (jac_solver_->JntToJac(q, J) < 0) return false;
    Eigen::MatrixXd Je = J.data;                    // 6 x n
    Eigen::VectorXd twist(6);
    twist << v[0], v[1], v[2], 0, 0, 0;
    Eigen::MatrixXd A = Je * Je.transpose() + dls_lambda_ * dls_lambda_ * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd x = Je.transpose() * A.ldlt().solve(twist);
    double scale = 1.0;
    for (size_t i = 0; i < n; ++i) scale = std::min(scale, max_joint_vel_ / std::max(1e-9, std::fabs(x(i))));
    qd.assign(n, 0.0);
    for (size_t i = 0; i < n; ++i) qd[i] = scale * x(i);
    return true;
  }

  void publishJoint(const Vec3 & v)
  {
    std_msgs::msg::Float64MultiArray m;
    std::vector<double> qd;
    if (!cartesianToJoints(v, qd)) qd.assign(joint_names_.size(), 0.0);
    m.data = qd;
    joint_pub_->publish(m);
  }

  void publishZero()
  {
    if (direct_) { publishJoint({0, 0, 0}); return; }
    geometry_msgs::msg::TwistStamped t;
    t.header.stamp = now();
    t.header.frame_id = command_frame_;
    twist_pub_->publish(t);
  }

  void tick()
  {
    if (!running_) return;
    if (ctrl_->status().state == State::DONE || ctrl_->status().state == State::FAILED) {
      publishZero();
      publishStatus();
      return;
    }
    if (!have_force_ || (now() - last_force_time_).seconds() > force_timeout_) {
      if (ctrl_->status().state != State::TARE) ctrl_->fail("force sensor timeout");
      publishZero();
      return;
    }
    // Force: sign correction, rotate sensor frame -> command frame
    geometry_msgs::msg::Vector3Stamped fs, fb;
    // The frame_id published by the Gazebo sensor bridge is not in the tf tree, so the
    // configured sensor_frame is used for the rotation (like the hand-guiding node does).
    fs.header.frame_id = sensor_frame_;
    fs.header.stamp = rclcpp::Time(0);   // latest available transform
    fs.vector.x = sign_[0] * last_wrench_.wrench.force.x;
    fs.vector.y = sign_[1] * last_wrench_.wrench.force.y;
    fs.vector.z = sign_[2] * last_wrench_.wrench.force.z;
    Vec3 pos{0, 0, 0};
    try {
      fb = tf_buffer_->transform(fs, command_frame_, tf2::durationFromSec(0.05));
      auto tf = tf_buffer_->lookupTransform(command_frame_, tool_frame_, tf2::TimePointZero);
      geometry_msgs::msg::PointStamped tip_in, tip;
      tip_in.header.frame_id = tool_frame_;
      tip_in.point.z = tool_tip_z_;
      tf2::doTransform(tip_in, tip, tf);
      pos = {tip.point.x, tip.point.y, tip.point.z};
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "tf not available: %s", e.what());
      publishZero();
      return;
    }
    const Vec3 f{fb.vector.x, fb.vector.y, fb.vector.z};
    Vec3 v = ctrl_->step(f, pos, 1.0 / rate_);
    const double sp = norm3(v);
    if (sp > max_speed_) for (auto & c : v) c *= max_speed_ / sp;

    if (direct_) publishJoint(v);
    geometry_msgs::msg::TwistStamped t;
    t.header.stamp = now();
    t.header.frame_id = command_frame_;
    t.twist.linear.x = v[0];
    t.twist.linear.y = v[1];
    t.twist.linear.z = v[2];
    if (!direct_) twist_pub_->publish(t);

    geometry_msgs::msg::PointStamped tp;
    tp.header.stamp = now();
    tp.header.frame_id = command_frame_;
    tp.point.x = pos[0]; tp.point.y = pos[1]; tp.point.z = pos[2];
    tip_pub_->publish(tp);
    publishStatus();
  }

  void publishStatus()
  {
    const auto & st = ctrl_->status();
    std_msgs::msg::Float64 m;
    m.data = st.fz;
    fz_pub_->publish(m);
    m.data = st.depth;
    depth_pub_->publish(m);
    std_msgs::msg::String s;
    s.data = stateName(st.state);
    state_pub_->publish(s);
    if (st.state != last_state_) {
      RCLCPP_INFO(get_logger(), "state %s -> %s %s", stateName(last_state_), stateName(st.state),
                  st.message.c_str());
      last_state_ = st.state;
    }
  }

  double sensor_delay_{0.0};
  std::deque<std::pair<rclcpp::Time, geometry_msgs::msg::WrenchStamped>> queue_;
  bool direct_{false};
  std::string joint_cmd_topic_;
  double dls_lambda_{0.02}, max_joint_vel_{0.5};
  KDL::Chain chain_;
  std::vector<std::string> joint_names_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::map<std::string, double> q_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  InsertionParams params_;
  std::array<double, 3> sign_{};
  std::string sensor_frame_, command_frame_, tool_frame_;
  double tool_tip_z_{0.0}, rate_{100.0}, force_timeout_{0.3}, max_speed_{0.01};
  bool auto_start_{true}, running_{false}, have_force_{false};
  State last_state_{State::TARE};
  geometry_msgs::msg::WrenchStamped last_wrench_;
  rclcpp::Time last_force_time_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<ScrewInsertion> ctrl_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fz_pub_, depth_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr tip_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScrewInsertionNode>());
  rclcpp::shutdown();
  return 0;
}
