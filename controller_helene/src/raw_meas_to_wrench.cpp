#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

using std::placeholders::_1;

class RawMeasToWrench : public rclcpp::Node {
public:
    RawMeasToWrench() : Node("raw_meas_to_wrench") {
        // --- PARAMETER DECLARATION ---
        this->declare_parameter<std::string>("sensor_frame_id", "axis_6");
        this->declare_parameter<std::string>("raw_meas_topic", "raw_meas_vector");
        this->declare_parameter<std::string>("wrench_topic", "/force_torque_sensor");
        this->declare_parameter<double>("scale_factor", 100.0);

        // --- PARAMETER READING ---
        sensor_frame_id_ = this->get_parameter("sensor_frame_id").as_string();
        std::string raw_meas_topic = this->get_parameter("raw_meas_topic").as_string();
        std::string wrench_topic = this->get_parameter("wrench_topic").as_string();
        scale_factor_ = this->get_parameter("scale_factor").as_double();

        // Prevent division by zero
        if (scale_factor_ == 0.0) {
            RCLCPP_ERROR(this->get_logger(), "Scale factor cannot be zero. Setting default to 1.0.");
            scale_factor_ = 1.0;
        }

        // --- PUBLISHER AND SUBSCRIBER INITIALIZATION ---
        wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(wrench_topic, 10);
        raw_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            raw_meas_topic, 10, std::bind(&RawMeasToWrench::raw_callback, this, _1));

        RCLCPP_INFO(this->get_logger(),
            "RawMeasToWrench node started: '%s' -> '%s' (Frame: %s, Scale: %.2f)",
            raw_meas_topic.c_str(), wrench_topic.c_str(), sensor_frame_id_.c_str(), scale_factor_);
    }

private:
    void raw_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        geometry_msgs::msg::WrenchStamped wrench;
        wrench.header.stamp = this->now();
        wrench.header.frame_id = sensor_frame_id_;

        // Convert raw values to Newtons using the defined scale factor
        wrench.wrench.force.x = msg->x / scale_factor_;
        wrench.wrench.force.y = msg->y / scale_factor_;
        wrench.wrench.force.z = msg->z / scale_factor_;

        // Torques remain zero for 3-axis force sensor
        wrench.wrench.torque.x = 0.0;
        wrench.wrench.torque.y = 0.0;
        wrench.wrench.torque.z = 0.0;

        wrench_pub_->publish(wrench);
    }

    std::string sensor_frame_id_;
    double scale_factor_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr raw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RawMeasToWrench>());
    rclcpp::shutdown();
    return 0;
}