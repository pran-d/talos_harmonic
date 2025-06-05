#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class StatePublisher : public rclcpp::Node
{
public:
    StatePublisher()
    : Node("state_publisher")
    {
        // Create publisher for JointState messages
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        RCLCPP_INFO(this->get_logger(), "%s started", this->get_name());

        // Create a timer to call the publish function periodically
        timer_ = this->create_wall_timer(33ms, std::bind(&StatePublisher::publish_joint_state, this));
    }

private:
    void publish_joint_state()
    {
        auto joint_state = sensor_msgs::msg::JointState();

        joint_state.header.stamp = this->get_clock()->now();
        joint_state.name = {"head_1_joint"};
        joint_state.position = {0.0};  // Static joint position
        joint_state.velocity = {0.1};
        joint_pub_->publish(joint_state);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}
