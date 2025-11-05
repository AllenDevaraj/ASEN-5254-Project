#!/usr/bin/env python3

"""
Launch file for Panda pick-and-place demo in Ignition Gazebo Fortress

This launch file:
1. Starts Ignition Gazebo with the pick-and-place world
2. Spawns the Panda robot
3. Starts ros2_control controllers
4. Launches ROS-Gazebo bridge for topics
5. Starts robot_state_publisher

Author: ASEN-5254 Project
Date: 2025
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get package directories
    panda_ign_desc_dir = get_package_share_directory('panda_ign_description')
    
    # Set Gazebo resource path so it can find meshes
    os.environ['GZ_SIM_RESOURCE_PATH'] = panda_ign_desc_dir
    
    # Paths
    world_file = os.path.join(panda_ign_desc_dir, 'worlds', 'pick_and_place_ign.sdf')
    urdf_file = os.path.join(panda_ign_desc_dir, 'urdf', 'panda_sim.urdf.xacro')
    controllers_file = os.path.join(panda_ign_desc_dir, 'config', 'panda_controllers.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Process robot description
    robot_description_content = Command([
        'xacro ', urdf_file
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
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
    
    # Spawn robot in Ignition
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'panda',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Arm controller spawner
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Gripper controller spawner
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # ROS-Gazebo bridge for camera
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
    
    # Bridge for clock
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
        
        # Robot state publisher (needs to be early for spawn to work)
        robot_state_publisher,
        
        # Spawn robot after a delay
        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        ),
        
        # Start bridges
        bridge_clock,
        bridge_camera_rgb,
        bridge_camera_depth,
        
        # Start controllers after robot is spawned
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    joint_state_broadcaster_spawner,
                ]
            )
        ),
        
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    arm_controller_spawner,
                    gripper_controller_spawner,
                ]
            )
        ),
    ])

