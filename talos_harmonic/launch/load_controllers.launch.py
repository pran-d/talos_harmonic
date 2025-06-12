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
                controllers=['lfc'],
                param_file=Path(
                    get_package_share_directory('talos_harmonic'),
                    'controllers',
                    'lfc_parameters.yaml',
                ),
                activate=False,
            ),
            load_controllers(
                controllers=['jse'],
                param_file=Path(
                    get_package_share_directory('talos_harmonic'),
                    'controllers',
                    'lfc_parameters.yaml',
                ),
                activate=False,
            ),
        )
    )


