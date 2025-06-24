#!/usr/bin/env python

"""Launch file use to resume the physics on gz simulator."""

from launch import LaunchDescription

from talos_harmonic.launch import (
    gz_play
)

from itertools import (
    chain,
)

def generate_launch_description():
    """Play physics gz."""

    return LaunchDescription(
        chain(gz_play())
    )
