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
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ros_gz_bridge.actions import RosGzBridge
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
   
    return LaunchDescription([

        ExecuteProcess(
            cmd=['gz', 'sim', '--physics-engine', 'gz-physics7-bullet-featherstone-plugin', '-r', model],
            output='screen'
        ),

        RosGzBridge(
            bridge_name="controller_bridge",
            config_file=config,
        ),

        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}],
        #     arguments=[os.path.join(pkg_talos, 'models', 'talos', 'talos_reduced.urdf')]
        # ),

    ])