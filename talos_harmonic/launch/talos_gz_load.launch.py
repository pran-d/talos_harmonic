from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    # Define controller names
    controller_names = LaunchConfiguration('controllers')

    # Package share directory
    pkg_talos_harmonic = FindPackageShare('talos_harmonic').find('talos_harmonic')

    # Paths to the individual launch files
    robot_spawn_launch = os.path.join(pkg_talos_harmonic, 'launch', 'robot_spawn.launch.py')
    load_controllers_launch = os.path.join(pkg_talos_harmonic, 'launch', 'load_controllers.launch.py')
    switch_controllers_launch = os.path.join(pkg_talos_harmonic, 'launch', 'switch_controllers.launch.py')

    return LaunchDescription([
        # Launch robot_spawn
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_spawn_launch)
        ),

        # Launch load_controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(load_controllers_launch)
        ),
    ])
