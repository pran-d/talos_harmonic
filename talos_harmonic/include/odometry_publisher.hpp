#ifndef ODOMETRY_PUBLISHER_HPP
#define ODOMETRY_PUBLISHER_HPP

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace odometry_dummy_estimator {

class OdometryPublisher : public controller_interface::ControllerInterface
{
    public:
        controller_interface::CallbackReturn on_init() final;

        controller_interface::InterfaceConfiguration command_interface_configuration()
            const final;

        controller_interface::InterfaceConfiguration state_interface_configuration()
            const final;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state) final;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) final;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) final;

        controller_interface::return_type update(const rclcpp::Time& time,
                                                const rclcpp::Duration& period);

    protected:
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            state_ordered_interfaces_;
        std::vector<
            std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            command_ordered_interfaces_;
};
}

#endif