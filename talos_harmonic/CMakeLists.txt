cmake_minimum_required(VERSION 3.8)
project(talos_harmonic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Install launch files
install(DIRECTORY launch config src include
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
ament_package()

# -------------------------------------------------------- #
# -------------------------------------------------------- #
# Adding custom hardware interface
# -------------------------------------------------------- #
add_library(odometry_hw_interface SHARED
src/odometry_hardware_interface.cpp)

target_include_directories(odometry_hw_interface PUBLIC include)

ament_target_dependencies(odometry_hw_interface
  pluginlib
  hardware_interface
  nav_msgs
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface plugins/plugin_description.xml)

install(
TARGETS odometry_hw_interface
DESTINATION lib
)