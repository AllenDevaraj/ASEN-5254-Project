#!/usr/bin/env python3

"""Unified launch file for dual Panda robots with single MoveIt move_group."""

import os
import xacro
import xml.etree.ElementTree as ET
from xml.dom import minidom
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def prefix_urdf_elements(urdf_root, prefix):
    """Prefix all links and joints in a URDF root element."""
    link_map = {}
    joint_map = {}
    
    # Prefix all links (except world)
    # Replace "panda_" prefix with new prefix (e.g., "panda_link0" -> "panda1_link0")
    for link in urdf_root.findall('link'):
        old_name = link.get('name')
        if old_name and old_name != 'world':
            # If link starts with "panda_", replace it with new prefix
            if old_name.startswith('panda_'):
                new_name = old_name.replace('panda_', prefix, 1)  # Replace only first occurrence
            else:
                # Otherwise, just prepend the prefix
                new_name = f"{prefix}{old_name}"
            link_map[old_name] = new_name
            link.set('name', new_name)
    
    # Prefix all joints
    # Replace "panda_" prefix with new prefix (e.g., "panda_joint1" -> "panda1_joint1")
    for joint in urdf_root.findall('joint'):
        old_name = joint.get('name')
        if old_name:
            # If joint starts with "panda_", replace it with new prefix
            if old_name.startswith('panda_'):
                new_name = old_name.replace('panda_', prefix, 1)  # Replace only first occurrence
            else:
                # Otherwise, just prepend the prefix
                new_name = f"{prefix}{old_name}"
            joint_map[old_name] = new_name
            joint.set('name', new_name)
            
            # Update parent/child link references
            parent = joint.find('parent')
            if parent is not None:
                link_name = parent.get('link')
                if link_name and link_name in link_map:
                    parent.set('link', link_map[link_name])
            
            child = joint.find('child')
            if child is not None:
                link_name = child.get('link')
                if link_name and link_name in link_map:
                    child.set('link', link_map[link_name])
            
            # Update mimic joint references (URDF mimic attribute)
            mimic = joint.find('mimic')
            if mimic is not None:
                mimic_joint_name = mimic.get('joint')
                if mimic_joint_name and mimic_joint_name in joint_map:
                    mimic.set('joint', joint_map[mimic_joint_name])
    
    # Update ros2_control joint references
    for ros2_control in urdf_root.findall('ros2_control'):
        for joint in ros2_control.findall('joint'):
            old_name = joint.get('name')
            if old_name and old_name in joint_map:
                joint.set('name', joint_map[old_name])
            
            # Update mimic references
            for param in joint.findall('param'):
                if param.get('name') == 'mimic':
                    mimic_name = param.text
                    if mimic_name and mimic_name in joint_map:
                        param.text = joint_map[mimic_name]
    
    return link_map, joint_map


def combine_urdfs(panda1_root, panda2_root):
    """Combine two URDF root elements into a single unified URDF."""
    # Create new root
    combined = ET.Element('robot')
    combined.set('name', 'dual_panda')
    
    # Copy world link (only one needed)
    world_link = panda1_root.find('link[@name="world"]')
    if world_link is not None:
        combined.append(ET.fromstring(ET.tostring(world_link)))
    
    # Copy all links from both (except world)
    # After prefixing, links should have panda1_* or panda2_* names
    for link in panda1_root.findall('link'):
        link_name = link.get('name')
        if link_name and link_name != 'world':
            # Deep copy the element (preserves all attributes including prefixed name)
            link_copy = ET.fromstring(ET.tostring(link))
            combined.append(link_copy)
            # Debug: verify name is preserved
            if 'link0' in link_name:
                print(f"[COMBINE DEBUG] Copied panda1 link: {link_copy.get('name')}")
    
    for link in panda2_root.findall('link'):
        link_name = link.get('name')
        if link_name and link_name != 'world':
            link_copy = ET.fromstring(ET.tostring(link))
            combined.append(link_copy)
            if 'link0' in link_name:
                print(f"[COMBINE DEBUG] Copied panda2 link: {link_copy.get('name')}")
    
    # Copy all joints from both
    for joint in panda1_root.findall('joint'):
        combined.append(ET.fromstring(ET.tostring(joint)))
    
    for joint in panda2_root.findall('joint'):
        new_joint = ET.fromstring(ET.tostring(joint))
        # Update panda2_joint_world origin for positioning
        if new_joint.get('name') == 'panda2_joint_world':
            origin = new_joint.find('origin')
            if origin is not None:
                # Panda2: x=1.4, y=-0.15, z=0.0, Yaw=pi
                origin.set('xyz', '1.4 -0.15 0.0')
                origin.set('rpy', '0 0 3.14159')
        combined.append(new_joint)
    
    # Copy ros2_control blocks with renamed identifiers
    ros2_control1 = panda1_root.find('ros2_control')
    if ros2_control1 is not None:
        new_ros2_control = ET.fromstring(ET.tostring(ros2_control1))
        new_ros2_control.set('name', 'panda1_arm')
        combined.append(new_ros2_control)
    
    ros2_control2 = panda2_root.find('ros2_control')
    if ros2_control2 is not None:
        new_ros2_control = ET.fromstring(ET.tostring(ros2_control2))
        new_ros2_control.set('name', 'panda2_arm')
        combined.append(new_ros2_control)
    
    # Copy gazebo plugins (only from first)
    for gazebo in panda1_root.findall('gazebo'):
        combined.append(ET.fromstring(ET.tostring(gazebo)))
    
    return combined


def generate_unified_robot_description():
    """Generate a unified robot_description by combining both Panda URDFs with prefixes."""
    panda_ign_desc_dir = get_package_share_directory('panda_ign_description')
    panda_urdf_file = os.path.join(panda_ign_desc_dir, 'urdf', 'panda_sim.urdf.xacro')
    controllers_file = os.path.join(panda_ign_desc_dir, 'config', 'panda_dual_controllers.yaml')
    
    # Process Panda 1 URDF with xacro
    print(f"[URDF GEN] Processing Panda1 xacro from: {panda_urdf_file}")
    panda1_doc = xacro.process_file(
        panda_urdf_file,
        mappings={
            'namespace': 'panda1',
            'controller_config': controllers_file
        }
    )
    # Convert to XML string then parse with ElementTree
    panda1_xml_str = panda1_doc.toxml()
    print(f"[URDF GEN] Panda1 XML length: {len(panda1_xml_str)} chars")
    panda1_root = ET.fromstring(panda1_xml_str)
    print(f"[URDF GEN] Panda1 root tag: {panda1_root.tag}, name: {panda1_root.get('name')}")
    
    # Check links before prefixing
    initial_links = [link.get('name') for link in panda1_root.findall('link')]
    print(f"[URDF GEN] Panda1 initial links (first 5): {initial_links[:5]}")
    
    # Process Panda 2 URDF with xacro
    print(f"[URDF GEN] Processing Panda2 xacro...")
    panda2_doc = xacro.process_file(
        panda_urdf_file,
        mappings={
            'namespace': 'panda2',
            'controller_config': controllers_file
        }
    )
    # Convert to XML string then parse with ElementTree
    panda2_xml_str = panda2_doc.toxml()
    panda2_root = ET.fromstring(panda2_xml_str)
    print(f"[URDF GEN] Panda2 root tag: {panda2_root.tag}, name: {panda2_root.get('name')}")
    
    # Debug: Check what links exist before prefixing
    panda1_links_before = [link.get('name') for link in panda1_root.findall('link')]
    print(f"[URDF GEN DEBUG] Panda1 links before prefixing: {panda1_links_before[:5]}...")
    
    # Prefix Panda 1 elements
    link_map1, joint_map1 = prefix_urdf_elements(panda1_root, 'panda1_')
    print(f"[URDF GEN DEBUG] Panda1 link_map entries: {len(link_map1)} links prefixed")
    print(f"[URDF GEN DEBUG] Panda1 link_map sample: {list(link_map1.items())[:3]}")
    
    # Prefix Panda 2 elements
    link_map2, joint_map2 = prefix_urdf_elements(panda2_root, 'panda2_')
    print(f"[URDF GEN DEBUG] Panda2 link_map entries: {len(link_map2)} links prefixed")
    
    # Verify prefixing worked - check directly in the ElementTree
    panda1_links_after = [link.get('name') for link in panda1_root.findall('link')]
    link0_after = [l for l in panda1_links_after if 'link0' in l]
    print(f"[URDF GEN DEBUG] Panda1 links after prefixing: {link0_after}")
    if 'panda1_link0' not in panda1_links_after:
        # Check if we got panda1_panda_link0 (wrong prefixing) vs panda1_link0 (correct)
        has_double_prefix = any('panda1_panda_link' in l for l in panda1_links_after)
        if has_double_prefix:
            raise ValueError(f"Prefixing created double prefix (panda1_panda_link0)! Links: {panda1_links_after[:10]}")
        else:
            raise ValueError(f"Prefixing failed! Panda1 links after prefixing: {panda1_links_after[:10]}")
    
    # Combine into unified URDF
    unified_root = combine_urdfs(panda1_root, panda2_root)
    
    # Verify combined root has prefixed links
    combined_links = [link.get('name') for link in unified_root.findall('link')]
    print(f"[URDF GEN DEBUG] Combined links sample: {[l for l in combined_links if 'link0' in l or 'link1' in l][:5]}")
    
    # Convert to pretty string
    rough_string = ET.tostring(unified_root, encoding='unicode')
    reparsed = minidom.parseString(rough_string)
    pretty = reparsed.toprettyxml(indent='  ')
    
    # Remove empty lines (keep first line which is XML declaration)
    lines = []
    first_line = True
    for line in pretty.split('\n'):
        stripped = line.strip()
        if stripped:
            lines.append(line)
        elif first_line and '<' in pretty.split('\n')[0]:
            # Keep XML declaration line even if empty
            first_line = False
    
    unified_urdf_string = '\n'.join(lines)
    
    # Debug: Check what's actually in the string
    print(f"[URDF GEN] Unified URDF generated: {len(unified_urdf_string)} characters")
    
    # Check for prefixed links in the string using regex
    import re
    link_matches = re.findall(r'<link name="([^"]+)">', unified_urdf_string)
    print(f"[URDF GEN] All link names found in string: {link_matches[:15]}")
    
    panda1_links_in_string = [l for l in link_matches if l.startswith('panda1_')]
    panda2_links_in_string = [l for l in link_matches if l.startswith('panda2_')]
    unprefixed_links = [l for l in link_matches if l.startswith('panda_link') and not l.startswith('panda1_') and not l.startswith('panda2_')]
    
    print(f"[URDF GEN] Panda1 links in string: {panda1_links_in_string[:5]}")
    print(f"[URDF GEN] Panda2 links in string: {panda2_links_in_string[:5]}")
    print(f"[URDF GEN] Unprefixed links in string: {unprefixed_links[:5]}")
    
    # Print debug info BEFORE validation
    print(f"[URDF GEN] Checking for panda1_link0 in string: {'panda1_link0' in unified_urdf_string}")
    print(f"[URDF GEN] Checking for panda2_link0 in string: {'panda2_link0' in unified_urdf_string}")
    
    # Validate URDF string contains expected elements
    if 'panda1_link0' not in unified_urdf_string:
        error_msg = f"Unified URDF missing panda1_link0 - prefixing failed!\n"
        error_msg += f"  Panda1 links found: {panda1_links_in_string[:10]}\n"
        error_msg += f"  Unprefixed links: {unprefixed_links[:10]}\n"
        error_msg += f"  Combined links from ET: {[l for l in combined_links if 'link0' in l or 'link1' in l][:5]}"
        error_msg += f"\n  Sample of unified_urdf_string (first 1000 chars): {unified_urdf_string[:1000]}"
        raise ValueError(error_msg)
    if 'panda2_link0' not in unified_urdf_string:
        error_msg = f"Unified URDF missing panda2_link0 - prefixing failed!\n"
        error_msg += f"  Panda2 links found: {panda2_links_in_string[:10]}"
        raise ValueError(error_msg)
    if '<robot name="dual_panda">' not in unified_urdf_string and '<robot name="dual_panda"' not in unified_urdf_string:
        raise ValueError("Unified URDF missing robot root element!")
    
    # Replace Gazebo mesh URIs with package:// URIs for MoveIt
    print("[URDF GEN] Fixing mesh paths for MoveIt collision geometry...")
    
    # Count meshes before fix
    model_uri_count = unified_urdf_string.count('model://panda/meshes/')
    print(f"[URDF GEN] Found {model_uri_count} Gazebo-style mesh URIs to fix")
    
    # Replace Gazebo URIs with package:// URIs
    unified_urdf_string = unified_urdf_string.replace(
        'model://panda/meshes/',
        'package://panda_ign_description/panda/meshes/'
    )
    
    # Verify fix was applied
    remaining_model_uris = unified_urdf_string.count('model://panda/meshes/')
    package_uris = unified_urdf_string.count('package://panda_ign_description/panda/meshes/')
    print(f"[URDF GEN] Mesh path fix complete: {package_uris} package:// URIs, {remaining_model_uris} model:// URIs remaining")
    
    if remaining_model_uris > 0:
        print("[URDF GEN] WARNING: Some model:// URIs were not replaced!")
    
    return ParameterValue(unified_urdf_string, value_type=str)


def generate_launch_description():
    """Launch everything needed for unified dual Panda control."""
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for MoveIt visualization'
    )
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch unified control GUI'
    )
    
    use_rviz = LaunchConfiguration('use_rviz')
    use_gui = LaunchConfiguration('use_gui')
    
    # Include the dual panda simulation launch (spawns robots separately)
    dual_panda_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panda_ign_description'),
                'launch',
                'dual_panda_demo.launch.py'
            ])
        ])
    )
    
    # Get directories
    panda_ign_desc_dir = get_package_share_directory('panda_ign_description')
    moveit_share = get_package_share_directory('moveit_resources_panda_moveit_config')
    
    # Generate unified robot_description (call function directly to get ParameterValue)
    print("[LAUNCH DEBUG] Generating unified URDF...")
    try:
        robot_description_param = generate_unified_robot_description()
        print("[LAUNCH DEBUG] Unified URDF generated successfully!")
    except Exception as e:
        print(f"[LAUNCH ERROR] Failed to generate unified URDF: {e}")
        import traceback
        traceback.print_exc()
        # Return empty launch description on error  
        return LaunchDescription([
            DeclareLaunchArgument('use_rviz', default_value='false'),
            DeclareLaunchArgument('use_gui', default_value='true'),
        ])
    
    # Build MoveIt config with unified SRDF and unified URDF
    # We need to manually set robot_description after building the config
    print("[DEBUG] Building MoveIt config...")
    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda', package_name='moveit_resources_panda_moveit_config')
        .robot_description_semantic(file_path='config/dual_panda.srdf')  # Unified SRDF (expects prefixed links)
        .robot_description_kinematics(file_path='config/kinematics.yaml')  # Same as original
        .planning_pipelines(pipelines=['ompl'])  # Same as original
        .trajectory_execution(file_path='config/dual_panda_moveit_controllers.yaml')  # Dual-arm controller config
        .to_moveit_configs()
    )
    
    # Override robot_description with our unified URDF
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict['robot_description'] = robot_description_param
    print("[LAUNCH DEBUG] MoveIt config built, unified URDF added to parameters.")
    print(f"[LAUNCH DEBUG] MoveIt config keys: {list(moveit_config_dict.keys())[:10]}...")  # Show first 10 keys
    
    # Joint state combiner: merges joint_states from both robots with prefixes
    # This node subscribes to /panda1/joint_states and /panda2/joint_states
    # and republishes as /joint_states with prefixed joint names (panda1_*, panda2_*)
    joint_state_combiner = ExecuteProcess(
        cmd=['python3', '-m', 'pick_and_place.joint_state_combiner'],
        output='screen'
    )
    
    # Unified Robot State Publisher
    # Publishes TF for the entire dual-arm system using the unified URDF and combined joint states
    # This ensures panda1_link0 etc. are published in the TF tree
    unified_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='unified_robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_param}],
        remappings=[('joint_states', '/joint_states')]
    )
    
    # Unified MoveIt move_group (no namespace)
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config_dict,  # Use modified config with unified URDF
            {'use_sim_time': True},
            {'planning_scene_monitor/publish_planning_scene': True},
            {'planning_scene_monitor/publish_geometry_updates': True},
            {'planning_scene_monitor/publish_state_updates': True},
        ],
        remappings=[
            ('joint_states', '/joint_states'),  # Use combined joint states
        ]
    )
    
    # Bridge for Ground Truth Poses (same as original)
    bridge_objects_global = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_objects_global',
        arguments=[
            '/world/pick_and_place_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ],
        remappings=[
            ('/world/pick_and_place_world/pose/info', '/objects_poses_sim')
        ],
        output='screen'
    )
    
    # Unified dual-arm GUI
    dual_arm_gui = ExecuteProcess(
        cmd=['python3', '-m', 'pick_and_place.dual_arm_single_group_gui'],
        output='screen',
        condition=IfCondition(use_gui)
    )
    
    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        use_gui_arg,
        
        # Bridges
        bridge_objects_global,
        
        # Simulation (must start first)
        dual_panda_sim,
        
        # Joint state combiner (after simulation starts, before MoveIt)
        TimerAction(
            period=3.0,
            actions=[joint_state_combiner, unified_rsp]
        ),
        
        # MoveIt move_group (after joint states are available - give combiner time to publish)
        TimerAction(
            period=15.0,  # Increased delay to ensure joint_state_combiner has published both arms
            actions=[move_group]
        ),
        
        # GUI (after MoveIt is ready)
        TimerAction(
            period=12.0,
            actions=[dual_arm_gui]
        ),
    ])

