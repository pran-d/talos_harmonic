#!/usr/bin/env python

"""Provide utils function to populate decription with robot_description."""

from collections.abc import (
    Generator,
    Mapping,
)
from pathlib import (
    Path,
)
from typing import (
    Dict,
    Optional,
    Text,
)

from launch import (
    Action,
)
from launch.actions import (
    DeclareLaunchArgument,
    SetLaunchConfiguration,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from launch_param_builder import (
    load_xacro,
)

from .invoke import (
    Invoke,
    SubstitutionOr,
)
from .logging import (
    logger,
)
from .utils import (
    dict_to_string,
)


def __make_mappings_from_string(txt: Text) -> Optional[Dict[Text, Text]]:
    from re import finditer

    mappings = {}
    for token in finditer(r'(\w+)=(\w+)', txt):
        mappings[token.group(1)] = token.group(2)

    return mappings if len(mappings) > 0 else None


def __write_when_required(
        value: Text,
        *,
        file_path: Optional[Path] = None,
) -> Text:
    if file_path is not None:
        logger.info('Dumping robot_description to {}'.format(file_path))
        with open(file_path, 'w') as f:
            f.write(value)

    return value


def __load_xacro(
        file_path: Path,
        mappings: Optional[Dict[Text, Text]] = None
) -> Text:
    logger.info(
        (
            'Loading XACRO:'
            '\n- File "{file_path}"'
            '\n- Using mappings:'
            '\n{mappings}'
        ).format(
            file_path=file_path,
            mappings=dict_to_string(
                mappings,
                fmt='--> {k}: {v}',
            ) if mappings is not None else '--> <NONE>'
        )
    )

    return load_xacro(file_path, mappings=mappings)


def add_robot_description_from_xacro(
        *,
        file_path: Optional[SubstitutionOr[Path]] = None,
        mappings: Optional[SubstitutionOr[Mapping[Text, Text]]] = None,
        output_file: Optional[SubstitutionOr[Path]] = None,
) -> Generator[Action]:
    """Create a Configuration with the robot_description from a xacro.

    Parameters
    ----------
    file_path: Optional[SubstitutionOr[Path]]
      Path to the xacro file. If None, declare a mandatory Launch argument
      for it
    mappings: Optional[SubstitutionOr[Mapping[Text, Text]]]
      Mappings of the XACRO. If None, declare a launch argument for it.
    output_file: Optional[SubstitutionOr[Path]]
      If given, will write the content of robot_description to the given file

    Returns
    -------
    Generator[Action]
      All ROS launch Actions used to perform the needed task.
    """
    if file_path is None:
        yield DeclareLaunchArgument(
            'file_path',
            description=(
                'XACRO file path use to create the robot_description URDF'
            ),
        )

        file_path = Invoke(
            Path,
            LaunchConfiguration('file_path'),
        )

    if mappings is None:
        yield DeclareLaunchArgument(
            'mappings',
            description=(
                (
                    'List of "key=value" separated by anythings that is '
                    'not within [a-zA-Z0-9_]. '
                    'Example: '
                    "mappings:='k0=v0 k1=v1 k3 =v3 k4= v4 k5 = v5'"
                    '-> k/v 3,4 and 5 will be ignored.'
                )
            ),
            default_value='',
        )

        mappings = Invoke(
            __make_mappings_from_string,
            LaunchConfiguration('mappings'),
        )

    if output_file is None:
        yield DeclareLaunchArgument(
            'output_file',
            description=(
                'If not empty, a valid file name (will be created) used to'
                ' write the robot_description into'
            ),
            default_value='',
        )

        output_file = Invoke(
            lambda txt: None if txt == '' else Path(txt),
            LaunchConfiguration('output_file'),
        )

    yield SetLaunchConfiguration(
        name='robot_description',
        value=Invoke(
            __load_xacro,
            file_path,
            mappings,
        ).and_then(
            __write_when_required,
            file_path=output_file,
        )
    )
