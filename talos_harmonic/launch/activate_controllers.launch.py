#!/usr/bin/env python

"""Launch file use to switch ANY ros2 control controller."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ( 
    DeclareLaunchArgument,
    TimerAction
)
from launch.substitutions import LaunchConfiguration

from pathlib import (
    Path,
)

from ament_index_python.packages import get_package_share_directory

from talos_harmonic.launch import (
    switch_controllers,
    gz_play
)

from itertools import (
    chain,
)

def generate_launch_description():
    """Load controllers."""

    controllers = LaunchConfiguration('controllers')
    activate = LaunchConfiguration('activate') 
    use_sim_time = LaunchConfiguration('use_sim_time')

    pd_controller = Node(
        package="talos_mpc",
        executable="pd_controller",
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    model_predictive_controller = Node(
        package="talos_mpc",
        executable="mpc_ros_interface",
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    default_controllers = [
        "arm_right_1_joint_inertia_shaping_controller",
        "arm_right_2_joint_inertia_shaping_controller",
        "arm_right_3_joint_inertia_shaping_controller",
        "arm_right_4_joint_inertia_shaping_controller",
        "arm_right_5_joint_inertia_shaping_controller",
        "arm_right_6_joint_inertia_shaping_controller",
        "arm_right_7_joint_inertia_shaping_controller",
        "arm_left_1_joint_inertia_shaping_controller",
        "arm_left_2_joint_inertia_shaping_controller",
        "arm_left_3_joint_inertia_shaping_controller",
        "arm_left_4_joint_inertia_shaping_controller",
        "arm_left_5_joint_inertia_shaping_controller",
        "arm_left_6_joint_inertia_shaping_controller",
        "arm_left_7_joint_inertia_shaping_controller",
        "leg_right_1_joint_inertia_shaping_controller",
        "leg_right_2_joint_inertia_shaping_controller",
        "leg_right_3_joint_inertia_shaping_controller",
        "leg_right_4_joint_inertia_shaping_controller",
        "leg_right_5_joint_inertia_shaping_controller",
        "leg_right_6_joint_inertia_shaping_controller",
        "leg_left_1_joint_inertia_shaping_controller",
        "leg_left_2_joint_inertia_shaping_controller",
        "leg_left_3_joint_inertia_shaping_controller",
        "leg_left_4_joint_inertia_shaping_controller",
        "leg_left_5_joint_inertia_shaping_controller",
        "leg_left_6_joint_inertia_shaping_controller",
        "torso_1_joint_inertia_shaping_controller",
        "torso_2_joint_inertia_shaping_controller",
        "head_1_joint_inertia_shaping_controller",
        "head_2_joint_inertia_shaping_controller",
        # "gripper_right_joint_inertia_shaping_controller",
        # "gripper_left_joint_inertia_shaping_controller",
        "lfc",
        "jse",
    ]

    default_controllers_str = " ".join(default_controllers)

    return LaunchDescription([
        DeclareLaunchArgument(
            'controllers',
            default_value=default_controllers_str,
            description='Controllers to switch'
        ),
        DeclareLaunchArgument(
            'activate',
            default_value='True',
            description='Whether to activate or deactivate the controllers'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Whether to use sim_time or not'
        ),

        model_predictive_controller,
        # pd_controller,
        
        TimerAction(
            period=1.0,
            actions=[*chain(switch_controllers())]
        ),
        TimerAction(
            period=2.5,
            actions=[*chain(gz_play())]
        )
    ])