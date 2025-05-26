'''
File Name: talos_spawn.launch.py
Author: Pranav Debbad (pranav.debbad@laas.fr)
Date: May 15, 2025

Description: 
Launch file used to:
1. spawn TALOS in Gazebo (with bullet physics engine)
2. run ros-gz-bridge for torque control
3. run robot_state_publisher
'''

from itertools import (
    chain,
)

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from ros_gz_bridge.actions import RosGzBridge
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
        
    model = PathJoinSubstitution([
        FindPackageShare('talos_harmonic'),
        'models', 'talos', 'empty_talos_gz.sdf'
    ])

    config = PathJoinSubstitution([
        FindPackageShare('talos_harmonic'),
        'launch', 'controller_bridge.yaml'
    ])

    urdf_file = os.path.join(
        get_package_share_directory('talos_harmonic'),
        'models',
        'talos',
        'talos_reduced.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
   
    return LaunchDescription([

        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value='/home/pdebbad/ros_space/talos_harmonic/src/talos_harmonic/models'),

        SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value='/opt/ros/jazzy/lib'),

        ExecuteProcess(
            cmd=['gz', 'sim', '--physics-engine', 'gz-physics7-bullet-featherstone-plugin', '-r', model],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # RosGzBridge(
        #     bridge_name="controller_bridge",
        #     config_file=config,
        # ),

        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}],
        #     arguments=[os.path.join(pkg_talos, 'models', 'talos', 'talos_reduced.urdf')]
        # ),

    ])