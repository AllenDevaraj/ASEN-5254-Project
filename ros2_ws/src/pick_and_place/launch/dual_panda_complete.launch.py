#!/usr/bin/env python3

"""
Complete launch file for dual Panda robots with MoveIt and GUIs.
Launches:
- Dual Panda simulation in Gazebo
- MoveIt for both pandas (namespaced)
- Both control GUIs
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Launch everything needed for dual Panda control."""
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for MoveIt visualization (disabled by default for dual robots)'
    )
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch control GUIs for both pandas'
    )
    
    use_rviz = LaunchConfiguration('use_rviz')
    use_gui = LaunchConfiguration('use_gui')
    
    # Include the dual panda simulation launch
    dual_panda_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panda_ign_description'),
                'launch',
                'dual_panda_demo.launch.py'
            ])
        ])
    )
    
    # Get MoveIt config directory
    moveit_share = get_package_share_directory('moveit_resources_panda_moveit_config')
    
    # Build MoveIt config for Panda 1
    # Note: We use the robot_description from the simulation (published by robot_state_publisher)
    # The robot_description parameter will be remapped to use the simulation's URDF
    moveit_config_panda1 = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(
            file_path='config/panda.urdf.xacro',
            mappings={'ros2_control_hardware_type': 'mock_components'}
        )
        .robot_description_semantic(file_path='config/panda.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .to_moveit_configs()
    )
    
    # Build MoveIt config for Panda 2 (same config)
    moveit_config_panda2 = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(
            file_path='config/panda.urdf.xacro',
            mappings={'ros2_control_hardware_type': 'mock_components'}
        )
        .robot_description_semantic(file_path='config/panda.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .to_moveit_configs()
    )
    
    # MoveIt move_group node for Panda 1
    # Remap robot_description to use the one from simulation
    move_group_panda1 = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        namespace='panda1',
        output='screen',
        parameters=[
            moveit_config_panda1.to_dict(),
            {'use_sim_time': True}
        ],
        remappings=[
            ('joint_states', '/panda1/joint_states'),
            # Remap robot_description to use the simulation's URDF
            ('robot_description', '/panda1/robot_description'),
            # Ensure compute_ik service is in the namespace
            ('compute_ik', '/panda1/compute_ik'),
        ]
    )
    
    # MoveIt move_group node for Panda 2
    # Remap robot_description to use the one from simulation
    move_group_panda2 = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        namespace='panda2',
        output='screen',
        parameters=[
            moveit_config_panda2.to_dict(),
            {'use_sim_time': True}
        ],
        remappings=[
            ('joint_states', '/panda2/joint_states'),
            # Remap robot_description to use the simulation's URDF
            ('robot_description', '/panda2/robot_description'),
            # Ensure compute_ik service is in the namespace
            ('compute_ik', '/panda2/compute_ik'),
        ]
    )
    
    # RViz node (optional, for visualization)
    rviz_config = os.path.join(moveit_share, 'launch', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config_panda1.robot_description,
            moveit_config_panda1.robot_description_semantic,
            moveit_config_panda1.planning_pipelines,
            moveit_config_panda1.robot_description_kinematics,
            {'use_sim_time': True}
        ],
        condition=IfCondition(use_rviz)
    )
    
    # Unified dual-arm GUI
    dual_arm_gui = ExecuteProcess(
        cmd=['ros2', 'run', 'pick_and_place', 'dual_arm_gui'],
        output='screen',
        condition=IfCondition(use_gui)
    )
    
    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        use_gui_arg,
        
        # Simulation (must start first)
        dual_panda_sim,
        
        # MoveIt nodes (start after simulation is ready)
        TimerAction(
            period=8.0,  # Wait for simulation and controllers to be ready
            actions=[
                move_group_panda1,
                move_group_panda2,
            ]
        ),
        
        # RViz (optional)
        TimerAction(
            period=10.0,
            actions=[rviz_node]
        ),
        
        # Dual-arm GUI (start after MoveIt is ready)
        TimerAction(
            period=12.0,
            actions=[dual_arm_gui]
        ),
    ])

