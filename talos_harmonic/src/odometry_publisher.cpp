#include "odometry_publisher.hpp"

#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace odometry_dummy_estimator {

class OdometryPublisher : public controller_interface::ControllerInterface
{
    public:
        controller_interface::CallbackReturn on_init() final;

        controller_interface::InterfaceConfiguration command_interface_configuration() const {
            controller_interface::InterfaceConfiguration command_interfaces_config;
            return command_interfaces_config;
        }

        controller_interface::InterfaceConfiguration state_interface_configuration() const {
            controller_interface::InterfaceConfiguration state_interfaces_config;
            state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
            state_interfaces_config.names = {
                "odom/base_translation/x",
                "odom/base_translation/y",
                "odom/base_translation/z",
            };
            return state_interfaces_config;
        }

        controller_interface::CallbackReturn on_configure(
              const rclcpp_lifecycle::State& previous_state) {
            return controller_interface::CallbackReturn::SUCCESS;
        }

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) {
            // Check if we have access to all state interfaces.
            bool ret = controller_interface::get_ordered_interfaces(
                state_interfaces_, params_.state_interfaces, std::string(""),
                state_ordered_interfaces_);
            if (!ret ||
                params_.state_interfaces.size() != state_ordered_interfaces_.size()) {
                RCLCPP_ERROR(this->get_node()->get_logger(),
                            "Expected %zu state interfaces, got %zu",
                            params_.state_interfaces.size(),
                            state_ordered_interfaces_.size());
                return controller_interface::CallbackReturn::ERROR;
            }
        }

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

