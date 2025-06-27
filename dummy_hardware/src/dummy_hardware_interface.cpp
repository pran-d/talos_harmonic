#include "dummy_hardware/dummy_hardware_interface.hpp"

namespace dummy_hardware
{

std::vector<hardware_interface::InterfaceDescription> DummyHardwareInterface::export_unlisted_state_interface_descriptions()
{
    std::vector<hardware_interface::InterfaceDescription> state_interfaces;

    hardware_interface::InterfaceInfo state_1;
    state_1.name = "x";
    hardware_interface::InterfaceDescription interface_1("base_translation", state_1);

    hardware_interface::InterfaceInfo state_2;
    state_2.name = "y";
    hardware_interface::InterfaceDescription interface_2("base_translation", state_2);

    hardware_interface::InterfaceInfo state_3;
    state_3.name = "z";
    hardware_interface::InterfaceDescription interface_3("base_translation", state_3);

    state_interfaces.emplace_back(interface_1);
    state_interfaces.emplace_back(interface_2);
    state_interfaces.emplace_back(interface_3);

    return state_interfaces;
}

hardware_interface::CallbackReturn DummyHardwareInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // Resize state vectors based on expected vector size (in this case, 3: x,y,z)
    position_states_.resize(3, 0.0);
    velocity_states_.resize(3, 0.0);

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DummyHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DummyHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    dummy_hardware::DummyHardwareInterface,
    hardware_interface::SystemInterface
)