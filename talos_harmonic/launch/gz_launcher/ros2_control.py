#!/usr/bin/env python

"""Add utils launch function managing ros_control stuff."""
from collections.abc import (
    Iterable,
    Generator,
)
from pathlib import (
    Path,
)
from typing import (
    List,
    Optional,
    Text,
)

from launch import (
    Action,
)
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import (
    Node,
)

from .invoke import (
    Invoke,
    SubstitutionOr,
)
from .logging import (
    logger,
)


def __make_spawner_args(
        controllers: Iterable[Text],
        activate: bool,
        param_file: Optional[Path] = None,
        controller_manager: Optional[Text] = None,
) -> List[Text]:

    args = [
        str(name) for name in controllers
    ]

    logger.info('Try to load controller(s) {}'.format(args))

    if param_file is not None:
        logger.info('--> With param file:\n"{}"'.format(param_file))
        args += [
            '--param-file',
            '{}'.format(param_file),
        ]

    if not activate:
        logger.info('--> Inactive by default')
        args.append('--inactive')

    if controller_manager is not None:
        logger.info(
            'Targeting controller_manager \"{}\"'.format(controller_manager)
        )
        args += [
            '-c',
            controller_manager,
        ]

    return args


def load_controllers(
        *,
        controllers: Optional[SubstitutionOr[Iterable[Text]]] = None,
        param_file: Optional[SubstitutionOr[Path]] = None,
        activate: Optional[SubstitutionOr[bool]] = None,
        controller_manager: Optional[SubstitutionOr[Text]] = None,
) -> Generator[Action]:
    """Load controllers into the controller manager.

    Parameters
    ----------
    controllers: Optional[SubstitutionOr[Iterable[Text]]]
        Mandatory list of controllers name we wish to load. If None, declare a
        launch argument for it.
    param_file: Optional[SubstitutionOr[Path]]
        YAML file path containing additional parameters associated to the
        controller(s) we wish to load. If None, declare a launch argument for
        it.
    activate: Optional[SubstitutionOr[bool]]
        Set to TRUE if you wish to directly activate the controller after
        load (the default let the controller configured but inactive). If None,
        declare a launch argument for it (default to False).
    controller_manager: Optional[SubstitutionOr[Text]]
        Name of the controller manager we wish to load controllers in. If None,
        declare a launch argument for it (default to '/controller_manager').

    Returns
    -------
    Generator[Action]
      All ROS launch Actions used to perform the needed task.
    """
    if controllers is None:
        yield DeclareLaunchArgument(
            'controllers',
            description=(
                'List of whitespace separated of name(s) corresponding to '
                'the controller(s) we wish to load'
            ),
        )
        controllers = Invoke(
            lambda txt: txt.split(' '),
            LaunchConfiguration('controllers'),
        )

    if param_file is None:
        yield DeclareLaunchArgument(
            'param_file',
            description=(
                'Path to the YAML controller parameter file'
            ),
            default_value='',
        )
        param_file = Invoke(
            lambda txt: Path(txt) if txt != '' else None,
            LaunchConfiguration('param_file'),
        )

    if activate is None:
        yield DeclareLaunchArgument(
            'activate',
            description=(
                (
                    'True if you wish to directly activate the controller '
                    'when loading (default to False)'
                )
            ),
            default_value='False',
            choices=['False', 'True']
        )
        activate = Invoke(
            lambda txt: True if txt == 'True' else False,
            LaunchConfiguration('activate'),
        )

    if controller_manager is None:
        yield DeclareLaunchArgument(
            'controller_manager',
            description=(
                (
                    'Name of the controller manager we wish to load the '
                    'controllers in.'
                )
            ),
            default_value='',
        )
        controller_manager = Invoke(
            lambda txt: txt if txt != '' else None,
            LaunchConfiguration('controller_manager'),
        )

    yield Invoke(
        __make_spawner_args,
        controllers=controllers,
        activate=activate,
        param_file=param_file,
        controller_manager=controller_manager,
    ).and_then_with_key(
        'arguments',
        Node,
        package='controller_manager',
        executable='spawner',
        output='screen'
    )

    yield Invoke(
        logger.info,
        'Unload any controller using '
        '`ros2 controll unload_controller <NAME>`'
    )


def __make_switch_controllers_cmd(
        controllers: Iterable[Text],
        activate: bool,
        controller_manager: Optional[Text] = None,
) -> List[Text]:
    cmd = [
        'ros2',
        'control',
        'switch_controllers',
        '--activate' if activate else '--deactivate',
    ]

    for name in controllers:
        cmd.append(name)

    logger.info(
        '{} controllers: {}...'.format(
            'Activating' if activate else 'Deactivating',
            ' '.join(cmd[4:])
        )
    )

    if controller_manager is not None:
        logger.info(
            'Targeting controller_manager \"{}\"'.format(controller_manager)
        )
        cmd += [
            '-c',
            controller_manager,
        ]

    logger.debug('Command sent: {}'.format(cmd))
    return cmd


def switch_controllers(
        *,
        controllers: Optional[SubstitutionOr[Iterable[Text]]] = None,
        activate: Optional[SubstitutionOr[bool]] = None,
        controller_manager: Optional[SubstitutionOr[Text]] = None,
) -> Generator[Action]:
    """Activate/Deactivate controllers.

    Parameters
    ----------
    controllers: Optional[SubstitutionOr[Iterable[Text]]]
        Mandatory list of controllers name we wish to activate/deactivate. If
        None, declare a launch argument for it.
    activate: Optional[SubstitutionOr[bool]]
        True if you wish to activate the controllers, False to deactivate
        them. If None, declare a launch argument for it.
    controller_manager: Optional[SubstitutionOr[Text]]
        Name of the controller manager containing the controllers. If None,
        declare a launch argument for it (default to '/controller_manager').

    Returns
    -------
    Generator[Action]
      All ROS launch Actions used to perform the needed task.
    """
    if controllers is None:
        yield DeclareLaunchArgument(
            'controllers',
            description=(
                'List of whitespace separated of name(s) corresponding to '
                'the controller(s) we wish to activate/deactivate'
            ),
        )
        controllers = Invoke(
            lambda txt: txt.split(' '),
            LaunchConfiguration('controllers'),
        )

    if activate is None:
        yield DeclareLaunchArgument(
            'activate',
            description=(
                (
                    'True if you wish to activate the controllers, False '
                    'to deactivate them'
                )
            ),
            choices=['False', 'True']
        )
        activate = Invoke(
            lambda txt: True if txt == 'True' else False,
            LaunchConfiguration('activate'),
        )

    if controller_manager is None:
        yield DeclareLaunchArgument(
            'controller_manager',
            description=(
                (
                    'Name of the controller manager containing the '
                    'controllers listed'
                )
            ),
            default_value='',
        )
        controller_manager = Invoke(
            lambda txt: txt if txt != '' else None,
            LaunchConfiguration('controller_manager'),
        )

    yield Invoke(
        __make_switch_controllers_cmd,
        controllers=controllers,
        activate=activate,
        controller_manager=controller_manager,
    ).and_then_with_key(
        'cmd',
        ExecuteProcess,
    )
