#!/usr/bin/env python

"""Launch file use to switch ANY ros2 control controller."""

from launch import LaunchDescription

from talos_harmonic.launch import (
    switch_controllers,
    gz_play
)

from itertools import (
    chain,
)


def generate_launch_description():
    """Load controllers."""
    return LaunchDescription(
        chain(
            gz_play(),
            switch_controllers()
        )
    )
