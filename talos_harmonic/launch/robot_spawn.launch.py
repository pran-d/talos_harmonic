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
)
from launch.substitutions import (
    LaunchConfiguration,
)

from talos_harmonic.launch import (
    GzWorld,
    Invoke,
    add_robot_description_from_xacro,
    all_arguments_from_yaml,
    evaluate_dict,
    gz_control,
    gz_server,
    gz_spawn_entity,
    load_controllers,
    run_robot_state_publisher,
    # switch_controllers,
)

def generate_launch_description():
    """Launch talos within GZ."""
    start_gz = chain(
        (
            # Since we are simulating, use_sim_time is FORCED here
            SetLaunchConfiguration(
                'use_sim_time',
                'True',
            ),
        ),
        gz_server()
    )

    xacro_file = Path(
        get_package_share_directory('talos_description'),
        'robots',
        'talos_full_v2.urdf.xacro',
    )

    xacro_args = list(
        all_arguments_from_yaml(
            Path(
                get_package_share_directory('talos_harmonic'),
                'config',
                'talos_configuration.yaml',
            )
        )
    )

    urdf_file = xacro_file.with_suffix('.urdf')

    # Remove the file extension from world defined by gz_server()
    world = Invoke(
        lambda v: Path(v).stem,
        LaunchConfiguration('world'),
    )

    spawn_tiago = chain(
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
        gz_spawn_entity(
            model_path=urdf_file,
            name='talos',
            world=world,
            timeout_ms=1000,
            z_height=1.08,
        ),
    )

    return LaunchDescription(
        chain(
            start_gz,
            spawn_tiago,
            gz_control(
                world=world,
                step=GzWorld.Pause(),
                timeout_ms=1000,

                # This disable the DeclareArgument('reset', ...)
                reset=Invoke(lambda *args: None),
            )
        )
    )
