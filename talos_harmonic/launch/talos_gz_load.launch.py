from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

import os

def generate_launch_description():
    # Define controller names
    controller_names = LaunchConfiguration('controllers')

    # Package share directory
    pkg_talos_harmonic = FindPackageShare('talos_harmonic').find('talos_harmonic')

    # Paths to the individual launch files
    robot_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_talos_harmonic, 'launch', 'robot_spawn.launch.py')
        )
    )
    load_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_talos_harmonic, 'launch', 'load_controllers.launch.py')
        )
    )
    activate_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_talos_harmonic, 'launch', 'activate_controllers.launch.py')
        )
    )
    change_robot_config = ExecuteProcess(
        cmd=['ros2', 'run', 'gz_gep_tools', 'control_loop', 't'],
        output='screen'
    )

    return LaunchDescription([
        robot_spawn_launch,
        load_controllers_launch,
        # TimerAction(
        #     period=10.0,
        #     actions=[change_robot_config]
        # ),
    ])

