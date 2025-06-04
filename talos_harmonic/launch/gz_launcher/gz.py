#!/usr/bin/env python

"""Add utils launch function managing GZ stuff."""
from collections.abc import (
    Generator,
    Mapping,
)
from enum import (
    Flag,
    auto,
)
from pathlib import (
    Path,
)
from typing import (
    Optional,
    Text,
    Union,
)

from launch import (
    Action,
)
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
)

from launch_ros.actions import (
    Node,
)

from .invoke import (
    Invoke,
    SubstitutionOr,
    evaluate_dict,
)
from .logging import (
    logger,
)
from .utils import (
    dict_to_string,
)


def __bool_from_string(s: Text) -> Optional[bool]:
    if s == 'True':
        return True
    elif s == 'False':
        return False
    else:
        return None


def __log_then_forward_cmd(cmd):
    logger.debug(
        'Command: `{}`'.format(
            ' '.join(cmd)
        )
    )
    return cmd


def __make_sim_cmd(
        gui: bool,
        world: Text,
        envs: Mapping[Text, Text],
):
    logger.info(
        (
            'Creating GZ sim server using:'
            '\n- Gui: {gui}'
            '\n- World: {world}'
            '\n- Envs:'
            '\n{envs}'
        ).format(
            gui=gui,
            world=world,
            envs=dict_to_string(
                envs,
                fmt='--> {k}: {v}',
            ),
        )
    )

    cmd = [
        'gz',
        'sim',
        world,
    ]

    if not gui:
        cmd.append('-s')

    return cmd


def gz_server(
        *,
        world: Optional[SubstitutionOr[Path]] = None,
        gui: Optional[SubstitutionOr[bool]] = None,
) -> Generator[Action]:
    """Create/update a description to launch a gz sim server.

    Parameters
    ----------
    world: Optional[SubstitutionOr[Path]]
      If not None, correspond to the sdf file use to spawn the server with.
      When None, declare a LaunchArgument for it (default to 'empty.sdf').
    gui: Optional[SubstitutionOr[bool]]
      If not None, indicates if we wish to spawn the UI or only the server in
      background. When None, declare a LaunchArgument for it (default to True).
    description: Optional[LaunchDescription]
      LaunchDescription to use instead of creating a new one

    Returns
    -------
    Generator[Action]
      All ROS launch Actions used to perform the needed task.
    """
    if world is None:
        yield DeclareLaunchArgument(
            'world',
            description='sdf file of the world we wish to create',
            default_value='empty.sdf'
        )
        world = LaunchConfiguration('world')

    if gui is None:
        yield DeclareLaunchArgument(
            'gui',
            description=(
                'Set to false if you wish to disable the GUI and only '
                'launch the server in background'
            ),
            default_value='True',
            choices=['False', 'True'],
        )
        gui = Invoke(
            __bool_from_string,
            LaunchConfiguration('gui'),
        )

    all_env_arguments = {
        'resource_path': (
            'world/models sdf files',
            'GZ_SIM_RESOURCE_PATH'
        ),
        'system_plugin_path': (
            'system plugins',
            'GZ_SIM_SYSTEM_PLUGIN_PATH'
        ),
        'server_config_path': (
            'server configurations',
            'GZ_SIM_SERVER_CONFIG_PATH'
        ),
        'gui_plugin_path': (
            'GUI plugins',
            'GZ_SIM_GUI_PLUGIN_PATH'
        ),
        'gui_resource_path': (
            'GUI resource files (config files)',
            'GZ_SIM_GUI_RESOURCE_PATH'
        ),
    }

    for arg_name, details in all_env_arguments.items():
        descr, env_var = details
        yield DeclareLaunchArgument(
            arg_name,
            description=(
                'Contains paths to {descr}. Will be appended to {env_var}.'
            ).format(
                descr=descr,
                env_var=env_var,
            ),
            default_value='',
        )

        yield AppendEnvironmentVariable(
            env_var,
            LaunchConfiguration(arg_name),
        )

    yield Invoke(
        __make_sim_cmd,
        gui=gui,
        world=world,
        envs=evaluate_dict(
            {
                name: EnvironmentVariable(name)
                for _, name in all_env_arguments.values()
            }
        ),
    ).and_then(
        __log_then_forward_cmd,
    ).and_then_with_key(
        'cmd',
        ExecuteProcess
    )

    # # FIXME: Is this needed ?
    yield Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )


def __make_spawn_cmd(
        world: Text,
        name: Text,
        model_path: Path,
        timeout_ms: int,
):
    logger.info(
        (
            "Spawning '{name}' from '{path}'"
            '\n- Into world: {world}'
            '\n- Timeout: {timeout}ms'
        ).format(
            name=name,
            world=world,
            path=model_path,
            timeout=timeout_ms,
        )
    )

    return [
        'gz',
        'service',
        '-s', '/world/{}/create'.format(world),
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '{}'.format(timeout_ms),
        '--req',
        'name: "{name}", sdf_filename: "{path}"'.format(
            name=name,
            path=model_path
        )
    ]


def gz_spawn_entity(
        *,
        model_path: Optional[SubstitutionOr[Path]] = None,
        name: Optional[SubstitutionOr[Text]] = None,
        world: Optional[SubstitutionOr[Text]] = None,
        timeout_ms: Optional[SubstitutionOr[int]] = None,
) -> Generator[Action]:
    """Spawn a model, with a given name, into an already running GZ server.

    Parameters
    ----------
    model_path: Optional[SubstitutionOr[Path]]
      If not None, the model we wish to spawn. It may be either a
      .sdf or .urdf file.
      When None, declare a LaunchArgument for it.
    name: Optional[SubstitutionOr[Text]]
      If not None, the name of the entity spawned inside gz.
      When None, declare a LaunchArgument for it (default to 'tiago').
    world: Optional[SubstitutionOr[Path]]
      If not None, the GZ world we wish to spawn our model into.
      When None, declare a LaunchArgument for it (default to 'empty').
    timeout: Optional[SubstitutionOr[int]]
      If not None, the timeout in ms associated to the gz service request.
      When None, declare a LaunchArgument for it (default to 1000).

    Returns
    -------
    Generator[Action]
      All ROS launch Actions used to perform the needed task.
    """
    if model_path is None:
        yield DeclareLaunchArgument(
            'model_path',
            description=(
                'The model file to spawn. '
                'Expecting either an sdf or urdf file path (checking '
                'files extensions).'
            ),
        )
        model_path = Invoke(
            Path,
            LaunchConfiguration('model_path')
        )

    if name is None:
        yield DeclareLaunchArgument(
            'name',
            description='Name of the entity to spawn inside gz',
            default_value='tiago',
        )
        name = LaunchConfiguration('name')

    if world is None:
        yield DeclareLaunchArgument(
            'world',
            description=(
                'Name of the world we wish to spawn the entity into'
            ),
            default_value='empty',
        )
        world = LaunchConfiguration('world')

    if timeout_ms is None:
        yield DeclareLaunchArgument(
            'timeout_ms',
            description='Timeout associated to the gz request (in ms)',
            default_value='1000',
        )
        timeout_ms = Invoke(
            int,
            LaunchConfiguration('timeout_ms'),
        )

    yield Invoke(
        __make_spawn_cmd,
        world,
        name,
        model_path,
        timeout_ms,
    ).and_then(
        __log_then_forward_cmd
    ).and_then_with_key(
        'cmd',
        ExecuteProcess,
    )


class GzWorld:
    """Namespace for gz_control related args values."""

    class Step():
        """Indicates the number of Steps done by the sim."""

        def __repr__(self):
            """Repr of the Step."""
            return 'Stepping {} times'.format(self.__value)

        def __init__(self, v):
            """Create a Step with a given value (>= 0)."""
            if not isinstance(v, int) or (v < 0):
                raise AttributeError(
                    'Step must be initialize with an uint (> 0)'
                    ' (got type "{}" = {})'.format(type(v), v)
                )

            self.__value = v

        def make_request(self):
            """Create the gz service req to step."""
            if self.__value > 1:
                return 'multi_step: {}'.format(self.__value)
            else:
                return 'step: true'

    class Play():
        """Play the sim."""

        def __repr__(self):
            """Repr of Play."""
            return 'Playing sim'

        def make_request(self):
            """Create the gz service req to play."""
            return 'pause: false'

    class Pause():
        """Pause the sim."""

        def __repr__(self):
            """Repr of Pause."""
            return 'Pausing sim'

        def make_request(self):
            """Create the gz service req to pause."""
            return 'pause: true'

    class Reset(Flag):
        """Enum use to select the reset type."""

        Time = auto()
        Model = auto()
        All = Time | Model


def __make_world_control_cmd(
        world: Text,
        step: Optional[Union[GzWorld.Play, GzWorld.Pause, GzWorld.Step]],
        reset: Optional[GzWorld.Reset],
        seed: Optional[int],
        timeout_ms: int,
) -> Text:
    if all(arg is None for arg in (step, seed, reset)):
        raise AttributeError(
            'No arguments provided to gz_control (Require at least one of'
            ' "step", "seed" and/or "reset" argument)'
        )

    msg = 'On world "{}":'.format(world)

    cmd = [
        'gz',
        'service',
        '-s', '/world/{}/control'.format(world),
        '--reqtype', 'gz.msgs.WorldControl',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '{}'.format(timeout_ms),
        '--req',
    ]

    requests = []

    if step is not None:
        msg += '\n - {} '.format(repr(step))
        requests.append(step.make_request())

    if reset is not None:
        msg += '\n - Reseting ({})'.format(reset)
        requests.append(
            'reset: {{{}: true}}'.format(
                'model_only' if reset == GzWorld.Reset.Model
                else 'time_only' if reset == GzWorld.Reset.Time
                else 'all'
            )
        )

    if seed is not None:
        if seed < 0:
            raise AttributeError(
                '"seed" argument MUST be >= 0 (seed = {})'.format(seed)
            )

        msg += '\n - Setting seed to {}'.format(seed)
        requests.append(
            'seed: {}'.format(seed)
        )

    logger.info(msg)

    cmd.append(', '.join(requests))
    return cmd


def gz_control(
        *,
        world: Optional[SubstitutionOr[Text]] = None,
        step: Optional[SubstitutionOr[Union[GzWorld.Play, GzWorld.Pause, GzWorld.Step]]] = None,
        reset: Optional[SubstitutionOr[GzWorld.Reset]] = None,
        seed: Optional[SubstitutionOr[int]] = None,
        timeout_ms: Optional[SubstitutionOr[int]] = None,
) -> Generator[Action]:
    """Spawn a model, with a given name, into an already running GZ server.

    Parameters
    ----------
    world: Optional[SubstitutionOr[Path]]
      If not None, the GZ world we wish to spawn our model into.
      When None, declare a LaunchArgument for it (default to 'empty').
    step: Optional[SubstitutionOr[Union[GzWorld.Play, GzWorld.Pause, GzWorld.Step]]]
      Either Play/Pause or a step value.
      When None, declare a LaunchArgument for it (default to '' - unset).
    reset: Optional[SubstitutionOr[GzWorld.Reset]]
      If not None, set to one of the GzWorld.Reset flag (Time, Model or All).
      When None, declare a LaunchArgument for it (default to '' - unset).
    seed: Optional[SubstitutionOr[uint]]
      If not None, set the random seed used by the world.
      When None, declare a LaunchArgument for it (default to '' - unset).
    timeout: Optional[SubstitutionOr[int]]
      If not None, the timeout in ms associated to the gz service request.
      When None, declare a LaunchArgument for it (default to 1000).

    Returns
    -------
    Generator[Action]
      All ROS launch Actions used to perform the needed task.
    """
    if world is None:
        yield DeclareLaunchArgument(
            'world',
            description=(
                'Name of the world we wish to spawn the entity into'
            ),
            default_value='empty',
        )
        world = LaunchConfiguration('world')

    if step is None:
        yield DeclareLaunchArgument(
            'step',
            description=(
                'Either "Play", "Pause" or an int value (>= 0) corresponding '
                'to the number of steps we wish to do'
            ),
            default_value='',
        )
        step = Invoke(
            lambda txt:
            None if txt == ''
            else GzWorld.Play() if txt == 'Play'
            else GzWorld.Pause() if txt == 'Pause'
            else GzWorld.Step(int(txt)),
            LaunchConfiguration('step'),
        )

    if reset is None:
        yield DeclareLaunchArgument(
            'reset',
            description='Reset either the Time or the Model (or both).',
            choices=['', 'Time', 'Model', 'All'],
            default_value='',
        )
        reset = Invoke(
            lambda txt: None if txt == '' else GzWorld.Reset[txt],
            LaunchConfiguration('reset'),
        )

    if seed is None:
        yield DeclareLaunchArgument(
            'seed',
            description='The random seed used by the sim. (MUST BE >= 0)',
            default_value='',
        )
        seed = Invoke(
            lambda t: None if t == '' else int(t),
            LaunchConfiguration('seed'),
        )

    if timeout_ms is None:
        yield DeclareLaunchArgument(
            'timeout_ms',
            description='Timeout associated to the gz request (in ms)',
            default_value='1000',
        )
        timeout_ms = Invoke(
            int,
            LaunchConfiguration('timeout_ms'),
        )

    yield Invoke(
        __make_world_control_cmd,
        world=world,
        step=step,
        reset=reset,
        seed=seed,
        timeout_ms=timeout_ms,
    ).and_then(
        __log_then_forward_cmd
    ).and_then_with_key(
        'cmd',
        ExecuteProcess,
    )
