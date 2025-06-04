"""ROS 2 launch file that spawns a GZ entity into an already running GZ world,
and initializes joint positions from SRDF."""

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import xml.etree.ElementTree as ET
import os
import sys

sys.path.append(os.path.dirname(__file__))
from gz_launcher import gz_spawn_entity


def parse_initial_joint_positions(srdf_path: str, state_name: str = "home") -> dict:
    """Parse the SRDF file to extract joint values for the given group state."""
    joint_positions = {}
    if not os.path.exists(srdf_path):
        print(f"[WARN] SRDF file not found: {srdf_path}")
        return joint_positions

    try:
        tree = ET.parse(srdf_path)
        root = tree.getroot()

        for group_state in root.findall('group_state'):
            if group_state.get('name') == state_name:
                for joint in group_state.findall('joint'):
                    joint_positions[joint.get('name')] = float(joint.get('value'))
                break
    except Exception as e:
        print(f"[ERROR] Failed to parse SRDF: {e}")

    return joint_positions


def generate_launch_description():
    """Spawn the Talos robot and set initial joint states from SRDF."""
    pkg_talos = FindPackageShare('talos_harmonic')

    urdf_file_path = PathJoinSubstitution([
        pkg_talos,
        'models',
        'talos_description',
        'talos.urdf'
    ])

    srdf_file_path = os.path.join(
        pkg_talos.find('talos_harmonic'),
        'models',
        'talos_description',
        'talos.srdf'
    )

    urdf_file_path_str = os.path.join(
        pkg_talos.find('talos_harmonic'),
        'models',
        'talos_description',
        'talos.urdf'
    )

    # Get joint states from SRDF
    initial_joint_states = parse_initial_joint_positions(srdf_file_path, state_name="home")

    with open(urdf_file_path_str, 'r') as urdf_file:
        robot_description = urdf_file.read()
 
    return LaunchDescription([
        # Spawn the robot entity in GZ Harmonic
        *gz_spawn_entity(
            model_path=urdf_file_path,
            name="talos",
            timeout_ms=1000
        ),

        # Start the joint_state_publisher with initial joint values
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'joint_states': initial_joint_states,
            }],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        )
    ])
