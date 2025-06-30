cmake_minimum_required(VERSION 3.8)
project(dummy_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools

  hardware_interface
  controller_interface
  
  std_msgs
  control_msgs
  nav_msgs
  
  gz_ros2_control
  gz_sim_vendor
  gz-sim
  gz_plugin_vendor
  gz-plugin
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# ------------------------------------------------------------------------------#
# ------------------------------------------------------------------------------#
# Adding the dummy_hardware_interface
# ------------------------------------------------------------------------------#

add_library(
  dummy_hardware
  SHARED
  src/dummy_hardware_interface.cpp
)
target_compile_features(dummy_hardware PUBLIC cxx_std_17)
target_include_directories(dummy_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/dummy_hardware>
)
ament_target_dependencies(
  dummy_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface plugins/dummy_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include/dummy_hardware
)
install(TARGETS dummy_hardware
  EXPORT export_dummy_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(
  dummy_hardware_gz
  SHARED
  src/dummy_hardware_interface_gz.cpp
)
target_compile_features(dummy_hardware_gz PUBLIC cxx_std_17)
target_include_directories(dummy_hardware_gz PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/dummy_hardware_gz>
)
ament_target_dependencies(
  dummy_hardware_gz PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(dummy_hardware_gz PUBLIC gz-sim::gz-sim)


# Export hardware plugins
pluginlib_export_plugin_description_file(gz_ros2_control plugins/dummy_hardware_gz.xml)

install(
  DIRECTORY include/
  DESTINATION include/dummy_hardware_gz
)
install(TARGETS dummy_hardware_gz
  EXPORT export_dummy_hardware_gz
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# ------------------------------------------------------------------------------#
# ------------------------------------------------------------------------------#
# Adding the dummy_controller
# ------------------------------------------------------------------------------#
add_library(
  dummy_controller
  SHARED
  src/dummy_controller.cpp
)
target_compile_features(dummy_controller PUBLIC cxx_std_17)
target_include_directories(dummy_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/dummy_controller>
)
ament_target_dependencies(
  dummy_controller PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(controller_interface plugins/dummy_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/dummy_controller
)
install(TARGETS dummy_controller
  EXPORT export_dummy_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_dummy_hardware HAS_LIBRARY_TARGET)
ament_export_targets(export_dummy_hardware_gz HAS_LIBRARY_TARGET)
ament_export_targets(export_dummy_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
