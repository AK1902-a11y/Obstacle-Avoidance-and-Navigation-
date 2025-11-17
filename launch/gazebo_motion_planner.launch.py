#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Package Directories
    pkg_name = 'robot_obstacle_avoidance'
    pkg_share = get_package_share_directory(pkg_name)

    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_gazebo.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'simple_world.world')
    
    # Check if files exist
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    if not os.path.exists(world_file):
        raise FileNotFoundError(f"World file not found: {world_file}")
    
    # Process xacro file to generate URDF
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Launch Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Path smoothing parameters
    declare_samples = DeclareLaunchArgument(
        'samples',
        default_value='100',
        description='Number of samples per path segment for smoothing'
    )
    
    # Trajectory generation parameters
    declare_v_max = DeclareLaunchArgument(
        'v_max',
        default_value='0.6',
        description='Maximum linear velocity (m/s)'
    )
    
    declare_a_max = DeclareLaunchArgument(
        'a_max',
        default_value='0.5',
        description='Maximum linear acceleration (m/s²)'
    )
    
    declare_a_lat_max = DeclareLaunchArgument(
        'a_lat_max',
        default_value='0.8',
        description='Maximum lateral acceleration (m/s²)'
    )
    
    # Start Gazebo Server
    start_gazebo_server = ExecuteProcess(
        cmd=['gzserver', 
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen'
    )
    
    # Start Gazebo Client (with small delay after server)
    start_gazebo_client = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['gzclient'],
                output='screen'
            )
        ]
    )
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
            'publish_frequency': 30.0
        }]
    )
    
    # Spawn Robot Entity in Gazebo
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=[
            '-entity', 'diff_drive_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # Delayed spawn (wait for Gazebo to fully initialize)
    spawn_robot_delayed = TimerAction(
        period=4.0,
        actions=[spawn_robot_node]
    )
    
    # Path Smoother Node
    path_smoother_node = Node(
        package='robot_obstacle_avoidance',
        executable='path_smoother',
        name='path_smoother_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'samples': LaunchConfiguration('samples'),
        }]
    )
    
    # Delayed path smoother (wait for Gazebo to be ready)
    path_smoother_delayed = TimerAction(
        period=6.0,  # Start after robot is spawned
        actions=[path_smoother_node]
    )
    
    # Trajectory Generator Node
    trajectory_generator_node = Node(
        package='robot_obstacle_avoidance',
        executable='trajectory_generator',
        name='trajectory_generator_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'v_max': LaunchConfiguration('v_max'),
            'a_max': LaunchConfiguration('a_max'),
            'a_lat_max': LaunchConfiguration('a_lat_max'),
        }]
    )
    
    # Delayed trajectory generator (wait for path smoother to complete)
    # Note: Adjust this delay based on path smoother execution time
    trajectory_generator_delayed = TimerAction(
        period=10.0,  # Start after path smoother finishes (~4s processing)
        actions=[trajectory_generator_node]
    )
    
    # Create Launch Description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_samples)
    ld.add_action(declare_v_max)
    ld.add_action(declare_a_max)
    ld.add_action(declare_a_lat_max)
    
    # Add nodes/processes in order
    ld.add_action(start_gazebo_server)           # t=0s: Start Gazebo server
    ld.add_action(start_gazebo_client)           # t=2s: Start Gazebo client
    ld.add_action(robot_state_publisher_node)    # t=0s: Publish robot description
    ld.add_action(spawn_robot_delayed)           # t=4s: Spawn robot in Gazebo
    ld.add_action(path_smoother_delayed)         # t=6s: Generate smooth path
    ld.add_action(trajectory_generator_delayed)  # t=10s: Generate trajectory
    
    return ld
