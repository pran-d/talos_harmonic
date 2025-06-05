from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directories
    robot_pkg = get_package_share_directory('talos_harmonic')
    gazebo_pkg = get_package_share_directory('ros_gz_sim')

    # File paths
    sdf_path = os.path.join(robot_pkg, 'models', 'talos_description', 'talos.sdf')
    srdf_path = os.path.join(robot_pkg, 'models', 'talos_description', 'talos.srdf')

    # Robot Description
    with open(sdf_path, 'r') as sdf_file:
        robot_description_content = sdf_file.read()
    
    with open(srdf_path, 'r') as srdf_file:
        robot_description_semantic_content = srdf_file.read()

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description_content}
        ],
        arguments=[sdf_path],
    )

    state_publisher_node = Node(
        package='talos_harmonic',
        executable='state_publisher',
        name='state_publisher',
        output='screen',
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-v4 empty.sdf']}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'talos',
            '-topic', 'robot_description',
            '-z', '1.127'
        ],
        output='screen'
    )

    spawn_with_delay = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        robot_state_publisher_node,
        state_publisher_node,
        spawn_with_delay,
    ])
