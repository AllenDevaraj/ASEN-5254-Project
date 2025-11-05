#!/usr/bin/env python3

"""
Launch file for DUAL Panda robots facing each other across the workbench

Author: ASEN-5254 Project
Date: 2025
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, GroupAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get package directory
    panda_ign_desc_dir = get_package_share_directory('panda_ign_description')
    
    # Paths
    world_file = os.path.join(panda_ign_desc_dir, 'worlds', 'pick_and_place_ign.sdf')
    urdf_file = os.path.join(panda_ign_desc_dir, 'urdf', 'panda_sim.urdf.xacro')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot 1 description (left side)
    robot1_description_content = Command([
        'xacro ', urdf_file
    ])
    
    # Robot 2 description (right side)  
    robot2_description_content = Command([
        'xacro ', urdf_file
    ])
    
    # Start Ignition Gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Robot 1 state publisher (left side)
    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='panda1',
        output='screen',
        parameters=[
            {'robot_description': robot1_description_content},
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('/robot_description', '/panda1/robot_description')]
    )
    
    # Robot 2 state publisher (right side)
    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='panda2',
        output='screen',
        parameters=[
            {'robot_description': robot2_description_content},
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('/robot_description', '/panda2/robot_description')]
    )
    
    # Spawn Robot 1 (one side of workbench)
    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'panda1',
            '-topic', '/panda1/robot_description',
            '-x', '0.0',
            '-y', '0.8',
            '-z', '0.0',
            '-Y', '-1.5708',  # Face toward workbench center
        ],
        output='screen'
    )
    
    # Spawn Robot 2 (opposite side, facing robot 1)
    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'panda2',
            '-topic', '/panda2/robot_description',
            '-x', '1.4',
            '-y', '-0.8',
            '-z', '0.0',
            '-Y', '1.5708',  # Face toward workbench center
        ],
        output='screen'
    )
    
    # Camera bridges
    bridge_camera_rgb = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
        ],
        output='screen'
    )
    
    bridge_camera_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/depth/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
        ],
        output='screen'
    )
    
    # Clock bridge
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),
        
        # Start Ignition Gazebo
        ignition_gazebo,
        
        # Robot state publishers
        robot1_state_publisher,
        robot2_state_publisher,
        
        # Spawn robots after delay
        TimerAction(
            period=3.0,
            actions=[spawn_robot1]
        ),
        
        TimerAction(
            period=4.0,
            actions=[spawn_robot2]
        ),
        
        # Start bridges
        bridge_clock,
        bridge_camera_rgb,
        bridge_camera_depth,
    ])

