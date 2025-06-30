#ifndef DUMMY_HARDWARE_INTERFACE_GZ_HPP_
#define DUMMY_HARDWARE_INTERFACE_GZ_HPP_

#include <memory>
#include <string>
#include <vector>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <nav_msgs/msg/odometry.hpp>

/*
Class containing a dummy hardware interface for odometry (base linear translation + velocity)
to be taken from the Gz simulation through ROS2
*/
namespace dummy_hardware_gz
{
class DummyHardwareInterfaceGz : public gz_ros2_control::GazeboSimSystemInterface
{
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DummyHardwareInterfaceGz)

        std::vector<hardware_interface::InterfaceDescription> export_unlisted_state_interface_descriptions() override;

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
        bool initSim(
            rclcpp::Node::SharedPtr & model_nh,
            std::map<std::string, sim::Entity> & joints,
            const hardware_interface::HardwareInfo & hardware_info,
            sim::EntityComponentManager & _ecm,
            unsigned int update_rate) override;

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_topic_subscriber_;
        rclcpp::Node::SharedPtr node_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;
};
}

#endif