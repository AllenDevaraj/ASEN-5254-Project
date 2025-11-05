#!/usr/bin/env python3
"""
Simple dual Panda launch - Just visual robots, no controllers yet
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    panda_ign_desc_dir = get_package_share_directory('panda_ign_description')
    world_file = os.path.join(panda_ign_desc_dir, 'worlds', 'pick_and_place_ign.sdf')
    
    # Start Gazebo
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
        }.items()
    )
    
    # Use SDF file directly from moveit_resources for robot models
    # Spawn without URDF to avoid ros2_control issues
    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(get_package_share_directory('moveit_resources_panda_description'), 
                                 'urdf', 'panda.urdf'),
            '-name', 'panda1',
            '-x', '0.0',
            '-y', '0.8',
            '-z', '0.0',
            '-Y', '-1.5708',
        ],
        output='screen'
    )
    
    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(get_package_share_directory('moveit_resources_panda_description'), 
                                 'urdf', 'panda.urdf'),
            '-name', 'panda2',
            '-x', '1.4',
            '-y', '-0.8',
            '-z', '0.0',
            '-Y', '1.5708',
        ],
        output='screen'
    )
    
    return LaunchDescription([
        ignition_gazebo,
        TimerAction(period=3.0, actions=[spawn_robot1]),
        TimerAction(period=4.0, actions=[spawn_robot2]),
    ])

