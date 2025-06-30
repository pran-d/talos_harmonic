
#include "dummy_controller/dummy_controller.hpp"
#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dummy_controller
{

controller_interface::CallbackReturn DummyController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DummyController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_names_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration DummyController::state_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn DummyController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_interface_names_ = {};

  // pre-reserve command interfaces
  command_interfaces_.reserve(command_interface_names_.size());

  // The names should be in the same order as for command interfaces for easier matching
  reference_interface_names_ = command_interface_names_;
  // for any case make reference interfaces size of command interfaces
  reference_interfaces_.resize(
  reference_interface_names_.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DummyController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_names_, std::string(""), ordered_interfaces) ||
    command_interface_names_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      this->get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_names_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");

  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DummyController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

bool DummyController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

controller_interface::return_type DummyController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    if (!std::isnan(reference_interfaces_[i]))
    {
      command_interfaces_[i].set_value(reference_interfaces_[i]);
    }
  }

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
DummyController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  for (size_t i = 0; i < reference_interface_names_.size(); ++i)
  {
    reference_interfaces.push_back(
      hardware_interface::CommandInterface(
        get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i]));
  }

  return reference_interfaces;
}

controller_interface::return_type DummyController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{  
  return controller_interface::return_type::OK;
}

}  // namespace dummy_controller

PLUGINLIB_EXPORT_CLASS(
  dummy_controller::DummyController, controller_interface::ChainableControllerInterface)
