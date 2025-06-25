#ifndef ODOMETRY_HARDWARE_INTERFACE_HPP_
#define ODOMETRY_HARDWARE_INTERFACE_HPP_

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>

/*
Class containing a hardware interface for odometry (base linear translation + velocity)
to be taken from the Gz simulation through ROS2
*/
namespace odom
{
class OdometryHardwareInterface : public hardware_interface::SystemInterface
{
    public:

        /// @brief ctor
        OdometryHardwareInterface();
        
        /// @brief dtor
        virtual ~OdometryHardwareInterface();

        // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
        virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        // Implementing hardware_interface::SystemInterface
        virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};
}
#endif