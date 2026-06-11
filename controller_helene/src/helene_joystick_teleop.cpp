#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <cmath>

class HeleneForceTeleop : public rclcpp::Node {
public:
    HeleneForceTeleop() : Node("helene_force_teleop") {
        // Calibration parameters
        this->declare_parameter<double>("force_deadband", 0.5); // Value below which force is considered zero (Newton or raw ADC value)
        this->declare_parameter<double>("force_to_velocity_scale", 0.05); // Conversion gain
        this->declare_parameter<std::string>("frame_id", "base_link");

        this->get_parameter("force_deadband", deadband_);
        this->get_parameter("force_to_velocity_scale", scale_);
        this->get_parameter("frame_id", frame_id_);

        // Subscription to the force sensor topic 
        force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/helene/force_sensor", 10, std::bind(&HeleneForceTeleop::force_callback, this, std::placeholders::_1));

        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);
    }

private:
    void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = this->now();
        twist_msg->header.frame_id = frame_id_;

        // Extract force components Fx, Fy, Fz
        double fx = msg->wrench.force.x;
        double fy = msg->wrench.force.y;
        double fz = msg->wrench.force.z;

        // Apply Deadband to prevent ghost movements (PLA Drift)
        twist_msg->twist.linear.x = (std::abs(fx) > deadband_) ? fx * scale_ : 0.0;
        twist_msg->twist.linear.y = (std::abs(fy) > deadband_) ? fy * scale_ : 0.0;
        twist_msg->twist.linear.z = (std::abs(fz) > deadband_) ? fz * scale_ : 0.0;

        // Note: if the sensor is rotated, you can map the torque moments to twist.angular

        twist_pub_->publish(std::move(twist_msg));
    }

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    double deadband_;
    double scale_;
    std::string frame_id_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeleneForceTeleop>());
    rclcpp::shutdown();
    return 0;
}