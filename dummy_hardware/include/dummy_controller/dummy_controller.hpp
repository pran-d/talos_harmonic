#ifndef DUMMY_CONTROLLER__DUMMY_CONTROLLER_HPP_
#define DUMMY_CONTROLLER__DUMMY_CONTROLLER_HPP_

// system
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

/**
 * DummyController is a simple chainable controller that exposes unlisted reference interfaces.
 * This controller simply forwards the information from a given list of topics
 * to its own command interfaces without any modifications.
 */
namespace dummy_controller
{
using DataType = std_msgs::msg::Float64MultiArray;
class DummyController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:  
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  rclcpp::Subscription<DataType>::SharedPtr data_sub_;

  std::vector<std::string> reference_interface_names_;

  std::vector<std::string> command_interface_names_;
};
}  // namespace dummy_controller

#endif  // DUMMY_CONTROLLER__DUMMY_CONTROLLER_HPP_
