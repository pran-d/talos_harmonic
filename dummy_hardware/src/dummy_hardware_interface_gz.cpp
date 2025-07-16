#include "dummy_hardware_gz/dummy_hardware_interface_gz.hpp"

namespace dummy_hardware_gz
{

std::vector<hardware_interface::InterfaceDescription> DummyHardwareInterfaceGz::export_unlisted_state_interface_descriptions()
{
    std::vector<hardware_interface::InterfaceDescription> state_interfaces;

    hardware_interface::InterfaceInfo state_1;
    state_1.name = "x";
    hardware_interface::InterfaceDescription interface_1("dummy_estimator/base_translation", state_1);

    hardware_interface::InterfaceInfo state_2;
    state_2.name = "y";
    hardware_interface::InterfaceDescription interface_2("dummy_estimator/base_translation", state_2);

    hardware_interface::InterfaceInfo state_3;
    state_3.name = "z";
    hardware_interface::InterfaceDescription interface_3("dummy_estimator/base_translation", state_3);

    hardware_interface::InterfaceInfo state_4;
    state_4.name = "x";
    hardware_interface::InterfaceDescription interface_4("dummy_estimator/base_linear_velocity", state_4);

    hardware_interface::InterfaceInfo state_5;
    state_5.name = "y";
    hardware_interface::InterfaceDescription interface_5("dummy_estimator/base_linear_velocity", state_5);

    hardware_interface::InterfaceInfo state_6;
    state_6.name = "z";
    hardware_interface::InterfaceDescription interface_6("dummy_estimator/base_linear_velocity", state_6);

    state_interfaces.emplace_back(interface_1);
    state_interfaces.emplace_back(interface_2);
    state_interfaces.emplace_back(interface_3);
    state_interfaces.emplace_back(interface_4);
    state_interfaces.emplace_back(interface_5);
    state_interfaces.emplace_back(interface_6);

    return state_interfaces;
}


hardware_interface::CallbackReturn DummyHardwareInterfaceGz::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // Resize state vectors based on expected vector size (in this case, 3: x,y,z)
    position_states_.resize(3, 0.0);
    position_states_[2] = 1.08;
    velocity_states_.resize(3, 0.0);

    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=odometry_topic_subscriber"});
    node_ = rclcpp::Node::make_shared("_", options);

    odometry_topic_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/model/talos/odometry",
      rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr odom_state){
        // RCLCPP_INFO(this->get_logger(), "Received odometry: x = %.2f, y = %.2f, z = %.2f",
        //     odom_state->pose.pose.position.x, odom_state->pose.pose.position.y, odom_state->pose.pose.position.z);
        position_states_[0] = odom_state->pose.pose.position.x;
        position_states_[1] = odom_state->pose.pose.position.y;
        position_states_[2] = odom_state->pose.pose.position.z;
        velocity_states_[0] = odom_state->twist.twist.linear.x;
        velocity_states_[1] = odom_state->twist.twist.linear.y;
        velocity_states_[2] = odom_state->twist.twist.linear.z;
      }
    );

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyHardwareInterfaceGz::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyHardwareInterfaceGz::on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyHardwareInterfaceGz::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DummyHardwareInterfaceGz::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
  }

  this->set_state("dummy_estimator/base_translation/x", position_states_[0]);
  this->set_state("dummy_estimator/base_translation/y", position_states_[1]);
  this->set_state("dummy_estimator/base_translation/z", position_states_[2]);
  this->set_state("dummy_estimator/base_linear_velocity/x", velocity_states_[0]);
  this->set_state("dummy_estimator/base_linear_velocity/y", velocity_states_[1]);
  this->set_state("dummy_estimator/base_linear_velocity/z", velocity_states_[2]);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DummyHardwareInterfaceGz::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

bool DummyHardwareInterfaceGz::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, sim::Entity> & joints,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & _ecm,
  unsigned int update_rate)
{
  return true;
}
} // end namespace: dummy_hardware_gz

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    dummy_hardware_gz::DummyHardwareInterfaceGz,
    gz_ros2_control::GazeboSimSystemInterface
)