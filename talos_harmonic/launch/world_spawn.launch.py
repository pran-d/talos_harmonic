#!/usr/bin/env python

"""Completely launch talos inside GZ in one go."""
from itertools import (
    chain,
)
from pathlib import (
    Path,
)

from ament_index_python.packages import get_package_share_directory

from launch import (
    LaunchDescription,
)
from launch.actions import (
    SetLaunchConfiguration,
    DeclareLaunchArgument
)
from launch.substitutions import (
    LaunchConfiguration,
)
from talos_harmonic.launch import (
    GzWorld,
    Invoke,
    gz_control,
    gz_server,
    evaluate_dict,
    all_arguments_from_yaml,
    run_robot_state_publisher,
    add_robot_description_from_xacro,
    # switch_controllers,
)

def generate_launch_description():
    """Launch talos within GZ."""

    sdf_path = Path(
        get_package_share_directory('talos_harmonic'),
        'worlds',
        'empty_talos_gz.sdf',
    )
    declare_world_arg = DeclareLaunchArgument(
        'world',
        description='SDF file of the world to simulate',
        default_value=str(sdf_path)
    )
    world = LaunchConfiguration('world')

    start_gz = chain(
        (
            # Since we are simulating, use_sim_time is FORCED here
            SetLaunchConfiguration(
                'use_sim_time',
                'True',
            ),
        ),
        gz_server(
            world=world
        )
    )

    xacro_file = Path(
        get_package_share_directory('talos_description'),
        'robots',
        'talos_full_fake_grippers.urdf.xacro',
    )

    xacro_args = list(
        all_arguments_from_yaml(
            Path(
                get_package_share_directory('talos_harmonic'),
                'config',
                'talos_configuration_no_gripper.yaml',
            )
        )
    )

    urdf_file = xacro_file.with_suffix('.urdf')

    launch_state_pub = chain(
        xacro_args,
        add_robot_description_from_xacro(
            file_path=xacro_file,
            mappings=evaluate_dict(
                {
                    arg.name: LaunchConfiguration(arg.name)
                    for arg in xacro_args
                } | {
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }
            ),
            output_file=urdf_file,
        ),
        run_robot_state_publisher(
            robot_description=LaunchConfiguration('robot_description'),
            use_sim_time=LaunchConfiguration('use_sim_time'),
        ),
    )

    return LaunchDescription([
        declare_world_arg,
        *chain(
            start_gz,
            launch_state_pub,
        )
    ])

