#ifndef DUMMY_HARDWARE_INTERFACE_HPP_
#define DUMMY_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

/*
Class containing a dummy hardware interface for odometry (base linear translation + velocity)
to be taken from the Gz simulation through ROS2
*/
namespace dummy_hardware
{
class DummyHardwareInterface : public hardware_interface::SystemInterface
{
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DummyHardwareInterface)

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

    private:
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;
};
}
#endif