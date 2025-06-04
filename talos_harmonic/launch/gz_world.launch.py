import sys
import os

sys.path.append(os.path.dirname(__file__))

from launch import LaunchDescription

from gz_launcher import (
    gz_server,
)
"""ROS2 launch file launching a GZ server."""

def generate_launch_description():
    """Create a GZ server."""
    return LaunchDescription(gz_server())
