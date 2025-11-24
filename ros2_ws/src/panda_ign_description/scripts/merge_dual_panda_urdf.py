#!/usr/bin/env python3
"""
Helper script to merge two Panda URDFs with prefixes into a unified dual-arm URDF.
This is used by the unified launch file to create robot_description.
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def prefix_urdf(urdf_content: str, prefix: str, base_pose: dict) -> ET.Element:
    """
    Take a URDF XML string and prefix all link/joint names, then transform base pose.
    
    Args:
        urdf_content: String content of URDF file
        prefix: Prefix to add (e.g., 'panda1_')
        base_pose: Dict with x, y, z, yaw for base link placement
        
    Returns:
        Root ElementTree element of the prefixed URDF
    """
    root = ET.fromstring(urdf_content)
    
    # Create a mapping of old names to new names
    link_map = {}
    joint_map = {}
    
    # First pass: collect all links and joints, create mapping
    for link in root.findall('.//link'):
        old_name = link.get('name')
        if old_name:
            new_name = prefix + old_name if old_name != 'world' else 'world'
            link_map[old_name] = new_name
            link.set('name', new_name)
    
    for joint in root.findall('.//joint'):
        old_name = joint.get('name')
        if old_name:
            new_name = prefix + old_name
            joint_map[old_name] = new_name
            joint.set('name', new_name)
    
    # Second pass: update references in joints (parent/child)
    for joint in root.findall('.//joint'):
        parent = joint.find('parent')
        if parent is not None:
            old_parent = parent.get('link')
            if old_parent and old_parent in link_map:
                parent.set('link', link_map[old_parent])
        
        child = joint.find('child')
        if child is not None:
            old_child = child.get('link')
            if old_child and old_child in link_map:
                child.set('link', link_map[old_child])
    
    # Update world joint origin for base pose
    world_joint = None
    for joint in root.findall('.//joint'):
        if 'world' in joint.get('name', '').lower() or (joint.find('parent') is not None and 
                                                          joint.find('parent').get('link') == 'world'):
            world_joint = joint
            break
    
    if world_joint is not None:
        origin = world_joint.find('origin')
        if origin is not None:
            origin.set('xyz', f"{base_pose['x']} {base_pose['y']} {base_pose['z']}")
            origin.set('rpy', f"0 0 {base_pose.get('yaw', 0)}")
    
    # Update ros2_control joint names
    for ros2_control in root.findall('.//ros2_control'):
        for joint in ros2_control.findall('.//joint'):
            old_name = joint.get('name')
            if old_name and old_name in joint_map:
                joint.set('name', joint_map[old_name])
    
    # Update mimic joint references
    for mimic in root.findall('.//mimic'):
        old_joint = mimic.get('joint')
        if old_joint and old_joint in joint_map:
            mimic.set('joint', joint_map[old_joint])
    
    return root


def merge_dual_panda_urdf(panda1_urdf_path: str, panda2_urdf_path: str, 
                          panda1_pose: dict, panda2_pose: dict) -> str:
    """
    Merge two Panda URDFs into a single dual-arm URDF.
    
    Returns:
        XML string of merged URDF
    """
    # Read both URDFs
    with open(panda1_urdf_path, 'r') as f:
        panda1_content = f.read()
    
    with open(panda2_urdf_path, 'r') as f:
        panda2_content = f.read()
    
    # Create unified robot root
    unified_robot = ET.Element('robot', {'name': 'dual_panda'})
    
    # Add Gazebo plugin once
    gazebo_plugin = ET.SubElement(unified_robot, 'gazebo')
    pose_pub = ET.SubElement(gazebo_plugin, 'plugin', {
        'filename': 'gz-sim-pose-publisher-system',
        'name': 'gz::sim::systems::PosePublisher'
    })
    ET.SubElement(pose_pub, 'publish_link_pose').text = 'true'
    ET.SubElement(pose_pub, 'use_pose_vector_msg').text = 'false'
    ET.SubElement(pose_pub, 'static_publisher').text = 'false'
    ET.SubElement(pose_pub, 'static_update_frequency').text = '60'
    
    # Add world link
    world_link = ET.SubElement(unified_robot, 'link', {'name': 'world'})
    
    # Process Panda 1 with prefix
    panda1_root = prefix_urdf(panda1_content, 'panda1_', panda1_pose)
    # Remove world link from panda1 (we already have it)
    for link in list(panda1_root.findall('.//link')):
        if link.get('name') == 'world':
            panda1_root.remove(link)
            break
    
    # Process Panda 2 with prefix
    panda2_root = prefix_urdf(panda2_content, 'panda2_', panda2_pose)
    # Remove world link from panda2
    for link in list(panda2_root.findall('.//link')):
        if link.get('name') == 'world':
            panda2_root.remove(link)
            break
    
    # Add all elements from panda1 (except robot root tag)
    for elem in panda1_root:
        unified_robot.append(elem)
    
    # Add all elements from panda2
    for elem in panda2_root:
        unified_robot.append(elem)
    
    # Convert to string with pretty printing
    ET.indent(unified_robot, space='  ')
    return ET.tostring(unified_robot, encoding='unicode')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: merge_dual_panda_urdf.py <panda1_urdf> <panda2_urdf>")
        sys.exit(1)
    
    panda1_path = sys.argv[1]
    panda2_path = sys.argv[2]
    
    panda1_pose = {'x': 0.0, 'y': 0.15, 'z': 0.0, 'yaw': 0.0}
    panda2_pose = {'x': 1.4, 'y': -0.15, 'z': 0.0, 'yaw': 3.14159}
    
    merged_urdf = merge_dual_panda_urdf(panda1_path, panda2_path, panda1_pose, panda2_pose)
    print(merged_urdf)
