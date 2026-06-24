#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

using namespace std::chrono_literals;

class MockForceSensor : public rclcpp::Node {
public:
    MockForceSensor() : Node("mock_force_sensor"), start_time_(this->now()) {
        // Publisher on the topic where your control node expects the data
        publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/helene_force_broadcaster/wrench", 10);

        // Publish at 100 Hz (10 ms)
        timer_ = this->create_wall_timer(
            10ms, std::bind(&MockForceSensor::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Mock Force Sensor Started. Simulating Kriechverhalten (creep behavior)...");
    }

private:
    void timer_callback() {
        // Wait until we have at least one subscriber before starting the simulation
        if (!simulation_started_) {
            if (publisher_->get_subscription_count() > 0) {
                simulation_started_ = true;
                start_time_ = this->now();
                RCLCPP_INFO(this->get_logger(), "Contact detected! Starting force simulation...");
            } else {
                return; // No subscribers yet, skip publishing
            }
        }
        
        auto msg = geometry_msgs::msg::WrenchStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "sensor_link"; // The sensor's reference frame

        // Calculate the elapsed time in seconds since the node started
        double t = (this->now() - start_time_).seconds();

        double force_z = 0.0;

        // --- SIMULATION STATE MACHINE ---

        if (t < 5.0) {
            // Phase 1 (0-5s): The robot is moving down, no force detected
            force_z = 0.0 + random_noise();
        } 
        else if (t >= 5.0 && t < 20.0) {
            // Phase 2 (5-20s): Contact made! Simulating Creep (Kriechverhalten)
            // The actual physical force is 5.0 N, but the plastic sensor deforms
            // Creep formula: Base_force + Drift_Amplitude * (1 - e^(-time / Time_constant))
            double tempo_contatto = t - 5.0;
            double forza_base = 5.0;
            double deriva_max = 2.5; // The sensor will drift up to +2.5 N
            double tau = 3.0; // Time constant (how fast it deforms)
            
            force_z = forza_base + deriva_max * (1.0 - std::exp(-tempo_contatto / tau)) + random_noise();
        } 
        else {
            // Phase 3 (>20s): The screw dropped into the hole! The normal force collapses
            force_z = 0.5 + random_noise();
        }

        // Populate the message (for now we focus only on Z; X and Y remain noise)
        msg.wrench.force.x = random_noise();
        msg.wrench.force.y = random_noise();
        msg.wrench.force.z = force_z;

        rclcpp::QoS qos_profile(10);
        qos_profile.transient_local();

        publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/helene_force_broadcaster/wrench", qos_profile);
    }

    bool simulation_started_ = false;

    // Adds slight background noise to make it realistic
    double random_noise() {
        return ((double)rand() / RAND_MAX - 0.5) * 0.1; // Noise between -0.05 and +0.05 N
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
    rclcpp::Time start_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::srand(std::time(nullptr)); // Initialize the random number generator
    rclcpp::spin(std::make_shared<MockForceSensor>());
    rclcpp::shutdown();
    return 0;
}