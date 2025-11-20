#!/usr/bin/env python3

"""
Launch file for DUAL Panda robots - Simple copy of working single with 2nd robot added

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
    # Use the dual controllers config with wildcards
    controllers_file = os.path.join(panda_ign_desc_dir, 'config', 'panda_dual_controllers.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Process robot description for Panda 1 (with namespace)
    robot_description_panda1_content = Command([
        'xacro ', urdf_file, 
        ' namespace:=panda1',
        ' controller_config:=', controllers_file
    ])
    
    robot_description_panda1 = {'robot_description': robot_description_panda1_content}
    
    # Process robot description for Panda 2 (with namespace)
    robot_description_panda2_content = Command([
        'xacro ', urdf_file, 
        ' namespace:=panda2',
        ' controller_config:=', controllers_file
    ])
    
    robot_description_panda2 = {'robot_description': robot_description_panda2_content}
    
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
    
    # Spawn robot 1 in Ignition (left side, closer to center)
    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'panda1',
            '-topic', '/panda1/robot_description',
            '-x', '0.0',
            '-y', '0.15',
            '-z', '0.0',
            '-Y', '0.0',  # Face forward (positive X direction)
        ],
        output='screen'
    )
    
    # Spawn robot 2 in Ignition (right side, closer to center, facing panda1)
    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'panda2',
            '-topic', '/panda2/robot_description',
            '-x', '1.4',
            '-y', '-0.15',
            '-z', '0.0',
            '-Y', '3.14159',  # Face backward (negative X direction) toward panda1
        ],
        output='screen'
    )
    
    # Robot state publisher for Panda 1
    robot_state_publisher_panda1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='panda1',
        output='screen',
        parameters=[
            robot_description_panda1,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Robot state publisher for Panda 2
    robot_state_publisher_panda2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='panda2',
        output='screen',
        parameters=[
            robot_description_panda2,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Joint state broadcaster spawner for Panda 1
    joint_state_broadcaster_spawner_panda1 = Node(
        package='controller_manager',
        executable='spawner',
        namespace='panda1',
        arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager', '--controller-manager-timeout', '30'],
        parameters=[controllers_file],
        output='screen'
    )
    
    # Arm controller spawner for Panda 1
    arm_controller_spawner_panda1 = Node(
        package='controller_manager',
        executable='spawner',
        namespace='panda1',
        arguments=['panda_arm_controller', '--controller-manager', 'controller_manager', '--controller-manager-timeout', '30'],
        parameters=[controllers_file],
        output='screen'
    )
    
    # Gripper controller spawner for Panda 1
    gripper_controller_spawner_panda1 = Node(
        package='controller_manager',
        executable='spawner',
        namespace='panda1',
        arguments=['panda_gripper_controller', '--controller-manager', 'controller_manager', '--controller-manager-timeout', '30'],
        parameters=[controllers_file],
        output='screen'
    )
    
    # Joint state broadcaster spawner for Panda 2
    joint_state_broadcaster_spawner_panda2 = Node(
        package='controller_manager',
        executable='spawner',
        namespace='panda2',
        arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager', '--controller-manager-timeout', '30'],
        parameters=[controllers_file],
        output='screen'
    )
    
    # Arm controller spawner for Panda 2
    arm_controller_spawner_panda2 = Node(
        package='controller_manager',
        executable='spawner',
        namespace='panda2',
        arguments=['panda_arm_controller', '--controller-manager', 'controller_manager', '--controller-manager-timeout', '30'],
        parameters=[controllers_file],
        output='screen'
    )
    
    # Gripper controller spawner for Panda 2
    gripper_controller_spawner_panda2 = Node(
        package='controller_manager',
        executable='spawner',
        namespace='panda2',
        arguments=['panda_gripper_controller', '--controller-manager', 'controller_manager', '--controller-manager-timeout', '30'],
        parameters=[controllers_file],
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
        
        # Robot state publishers (needs to be early for spawn to work)
        robot_state_publisher_panda1,
        robot_state_publisher_panda2,
        
        # Spawn BOTH robots after a delay
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
        
        # Start controllers for Panda 1 after robot is spawned
        # Use OnProcessExit to ensure proper sequencing and avoid race conditions
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot1,
                on_exit=[
                    TimerAction(
                        period=3.0,  # Give controller manager time to initialize
                        actions=[joint_state_broadcaster_spawner_panda1]
                    )
                ]
            )
        ),
        
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner_panda1,
                on_exit=[
                    TimerAction(
                        period=2.0,  # Wait for joint_state_broadcaster to fully configure
                        actions=[
                            arm_controller_spawner_panda1,
                            gripper_controller_spawner_panda1,
                        ]
                    )
                ]
            )
        ),
        
        # Start controllers for Panda 2 after robot is spawned
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot2,
                on_exit=[
                    TimerAction(
                        period=3.0,  # Give controller manager time to initialize
                        actions=[joint_state_broadcaster_spawner_panda2]
                    )
                ]
            )
        ),
        
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner_panda2,
                on_exit=[
                    TimerAction(
                        period=2.0,  # Wait for joint_state_broadcaster to fully configure
                        actions=[
                            arm_controller_spawner_panda2,
                            gripper_controller_spawner_panda2,
                        ]
                    )
                ]
            )
        ),
    ])
