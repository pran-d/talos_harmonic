#include "odometry_hardware_interface.hpp"

odom::OdometryHardwareInterface::OdometryHardwareInterface(): position_states_({0.0}), velocity_states_({0.0}){}

odom::OdometryHardwareInterface::~OdometryHardwareInterface() = default;

hardware_interface::CallbackReturn odom::OdometryHardwareInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // Resize state vectors based on expected vector size (in this case, 3: x,y,z)
    position_states_.resize(3, 0.0);
    velocity_states_.resize(3, 0.0);

    // Create a node (not lifecycle node) for subscriptions
    node_ = std::make_shared<rclcpp::Node>("odometry_hw_interface_node");

    // Subscribe to odometry
    odom_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/model/talos/odometry", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            // Update state from Odometry message
            position_states_[0] = msg->pose.pose.position.x;
            position_states_[1] = msg->pose.pose.position.y;
            position_states_[2] = msg->pose.pose.position.z;
            velocity_states_[0] = msg->twist.twist.linear.x;
            velocity_states_[1] = msg->twist.twist.linear.y;
            velocity_states_[2] = msg->twist.twist.linear.z;
        }
    );

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn odom::OdometryHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(node_->get_logger(), "OdometryHardwareInterface activated.");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn odom::OdometryHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(node_->get_logger(), "OdometryHardwareInterface deactivated.");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> odom::OdometryHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "base_translation.x", hardware_interface::HW_IF_POSITION, &position_states_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "base_translation.y", hardware_interface::HW_IF_POSITION, &position_states_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "base_translation.z", hardware_interface::HW_IF_POSITION, &position_states_[2]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "base_linear_velocity.x", hardware_interface::HW_IF_VELOCITY, &velocity_states_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "base_linear_velocity.y", hardware_interface::HW_IF_VELOCITY, &velocity_states_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "base_linear_velocity.z", hardware_interface::HW_IF_VELOCITY, &velocity_states_[2]));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> odom::OdometryHardwareInterface::export_command_interfaces()
{
    // This interface is read-only; we don't export command interfaces, only state interfaces
    return {};
}

hardware_interface::return_type odom::OdometryHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Spin the node (non-blocking) to process the subscription callback
    rclcpp::spin_some(node_);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type odom::OdometryHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // No writing to hardware required
    return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(
    odom::OdometryHardwareInterface,
    hardware_interface::SystemInterface
)