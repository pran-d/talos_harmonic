"""Load the controllers LFC and JSE for Talos."""
from itertools import (
    chain,
)
from pathlib import (
    Path,
)

import os

from ament_index_python.packages import get_package_share_directory

from launch import (
    LaunchDescription,
)

from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from talos_harmonic.launch import (
    Invoke,
    load_controllers,
    # switch_controllers,
)

def generate_launch_description():
    """Launch LFC controller for Talos"""

    return LaunchDescription(
        chain(
            load_controllers(
                controllers=[
                    'arm_right_1_joint_inertia_shaping_controller',
                    # 'arm_right_2_joint_inertia_shaping_controller',
                    # 'arm_right_3_joint_inertia_shaping_controller',
                    # 'arm_right_4_joint_inertia_shaping_controller',
                    # 'arm_right_5_joint_inertia_shaping_controller',
                    # 'arm_right_6_joint_inertia_shaping_controller',
                    # 'arm_right_7_joint_inertia_shaping_controller', 
                    # 'arm_left_1_joint_inertia_shaping_controller',
                    # 'arm_left_2_joint_inertia_shaping_controller',
                    # 'arm_left_3_joint_inertia_shaping_controller',
                    # 'arm_left_4_joint_inertia_shaping_controller',
                    # 'arm_left_5_joint_inertia_shaping_controller',
                    # 'arm_left_6_joint_inertia_shaping_controller',
                    # 'arm_left_7_joint_inertia_shaping_controller',
                    # 'leg_right_1_joint_inertia_shaping_controller',
                    # 'leg_right_2_joint_inertia_shaping_controller',
                    # 'leg_right_3_joint_inertia_shaping_controller',
                    # 'leg_right_4_joint_inertia_shaping_controller',
                    # 'leg_right_5_joint_inertia_shaping_controller',
                    # 'leg_right_6_joint_inertia_shaping_controller',
                    # 'leg_left_1_joint_inertia_shaping_controller',
                    # 'leg_left_2_joint_inertia_shaping_controller',
                    # 'leg_left_3_joint_inertia_shaping_controller',
                    # 'leg_left_4_joint_inertia_shaping_controller',
                    # 'leg_left_5_joint_inertia_shaping_controller',
                    # 'leg_left_6_joint_inertia_shaping_controller',
                    # 'torso_1_joint_inertia_shaping_controller',
                    # 'torso_2_joint_inertia_shaping_controller',
                    # 'head_1_joint_inertia_shaping_controller',
                    # 'head_2_joint_inertia_shaping_controller',
                    # 'gripper_right_joint_inertia_shaping_controller',
                    # 'gripper_left_joint_inertia_shaping_controller',
                ],
                param_file=Path(
                    get_package_share_directory('talos_harmonic'),
                    'controllers',
                    'dummy_controllers.yaml',
                ),
                activate=False,
            ),
            # load_controllers(
            #     controllers=['lfc'],
            #     param_file=Path(
            #         get_package_share_directory('talos_harmonic'),
            #         'controllers',
            #         'lfc_parameters.yaml',
            #     ),
            #     activate=False,
            # ),
            # load_controllers(
            #     controllers=['jse'],
            #     param_file=Path(
            #         get_package_share_directory('talos_harmonic'),
            #         'controllers',
            #         'lfc_parameters.yaml',
            #     ),
            #     activate=False,
            # ),
        )
    )


