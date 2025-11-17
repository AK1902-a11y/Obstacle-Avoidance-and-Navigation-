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
    rviz_config_file = os.path.join(pkg_share, 'config', 'robot_con.rviz')

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
    
    declare_obstacle_threshold = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='0.8',
        description='Distance threshold to stop for obstacles (meters)'
    )
    
    declare_min_safe_distance = DeclareLaunchArgument(
        'min_safe_distance',
        default_value='0.6',
        description='Minimum safe clearance distance (meters)'
    )
    
    declare_forward_velocity = DeclareLaunchArgument(
        'forward_velocity',
        default_value='0.3',
        description='Forward exploration velocity (m/s)'
    )
    
    declare_turn_velocity = DeclareLaunchArgument(
        'turn_velocity',
        default_value='0.5',
        description='Angular velocity for turning (rad/s)'
    )
    
    # Start Gazebo Server
    start_gazebo_server = ExecuteProcess(
        cmd=['gzserver', 
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen'
    )
    
    # Start Gazebo Client (optional, with delay after server)
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
    
    spawn_robot_delayed = TimerAction(
        period=4.0,
        actions=[spawn_robot_node]
    )
    
    # Obstacle Avoidance Node (your exploration node)
    obstacle_avoidance_node = Node(
        package='robot_obstacle_avoidance',
        executable='obstacle_avoidance',  # ensure executable matches your setup.py entry
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'min_safe_distance': LaunchConfiguration('min_safe_distance'),
            'forward_velocity': LaunchConfiguration('forward_velocity'),
            'turn_velocity': LaunchConfiguration('turn_velocity'),
        }],
        remappings=[
            ('/scan', '/scan'),
            ('/cmd_vel', '/cmd_vel'),
            ('/camera/image_raw', '/camera/image_raw'),
        ]
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Delay start of exploration node to allow sensors to initialize
    obstacle_avoidance_delayed = TimerAction(
        period=8.0,
        actions=[obstacle_avoidance_node]
    )
    
    # Create Launch Description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_obstacle_threshold)
    ld.add_action(declare_min_safe_distance)
    ld.add_action(declare_forward_velocity)
    ld.add_action(declare_turn_velocity)
    
    # Add nodes/processes
    ld.add_action(start_gazebo_server)           # t=0s
    ld.add_action(start_gazebo_client)           # t=2s
    ld.add_action(robot_state_publisher_node)    # t=0s
    ld.add_action(rviz_node) 
    ld.add_action(spawn_robot_delayed)           # t=4s
    ld.add_action(obstacle_avoidance_delayed)    # t=8s
    
    return ld
