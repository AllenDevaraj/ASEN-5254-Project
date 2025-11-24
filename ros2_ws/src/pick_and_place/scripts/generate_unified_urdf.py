#!/usr/bin/env python3
"""
Generate unified URDF combining two Panda robots with prefixed names.

This script processes the base Panda URDF/Xacro and creates a unified URDF
with both robots prefixed (panda1_* and panda2_*).
"""

import os
import sys
import xml.etree.ElementTree as ET
import xacro
from xml.dom import minidom


def prefix_urdf_elements(urdf_root, prefix):
    """
    Prefix all links and joints in a URDF root element.
    Updates references throughout the URDF.
    """
    link_map = {}
    joint_map = {}
    
    # Prefix all links (except world)
    for link in urdf_root.findall('link'):
        old_name = link.get('name')
        if old_name and old_name != 'world':
            new_name = f"{prefix}{old_name}"
            link_map[old_name] = new_name
            link.set('name', new_name)
    
    # Prefix all joints
    for joint in urdf_root.findall('joint'):
        old_name = joint.get('name')
        if old_name:
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
    """
    Combine two URDF root elements into a single unified URDF.
    """
    # Create new root
    combined = ET.Element('robot')
    combined.set('name', 'dual_panda')
    
    # Copy world link (only one needed)
    world_link = panda1_root.find('link[@name="world"]')
    if world_link is not None:
        combined.append(world_link)
    
    # Copy all links from both (except world)
    for link in panda1_root.findall('link'):
        if link.get('name') != 'world':
            combined.append(link)
    for link in panda2_root.findall('link'):
        if link.get('name') != 'world':
            combined.append(link)
    
    # Copy all joints from both
    for joint in panda1_root.findall('joint'):
        combined.append(joint)
    
    for joint in panda2_root.findall('joint'):
        # Update panda2_joint_world origin for positioning
        if joint.get('name') == 'panda2_joint_world':
            origin = joint.find('origin')
            if origin is not None:
                # Panda2: x=1.4, y=-0.15, z=0.0, Yaw=pi
                origin.set('xyz', '1.4 -0.15 0.0')
                origin.set('rpy', '0 0 3.14159')
        combined.append(joint)
    
    # Copy ros2_control blocks with renamed identifiers
    ros2_control1 = panda1_root.find('ros2_control')
    if ros2_control1 is not None:
        ros2_control1.set('name', 'panda1_arm')
        combined.append(ros2_control1)
    
    ros2_control2 = panda2_root.find('ros2_control')
    if ros2_control2 is not None:
        ros2_control2.set('name', 'panda2_arm')
        combined.append(ros2_control2)
    
    # Copy gazebo plugins (only from first, or handle separately)
    for gazebo in panda1_root.findall('gazebo'):
        combined.append(gazebo)
    
    return combined


def generate_unified_urdf_string(urdf_xacro_path, controllers_file):
    """
    Generate unified URDF string from xacro file.
    
    Args:
        urdf_xacro_path: Path to panda_sim.urdf.xacro
        controllers_file: Path to controller config
    
    Returns:
        Unified URDF as XML string
    """
    # Process Panda 1 URDF with xacro
    panda1_doc = xacro.process_file(
        urdf_xacro_path,
        mappings={
            'namespace': 'panda1',
            'controller_config': controllers_file
        }
    )
    panda1_root = panda1_doc.getroot()
    
    # Process Panda 2 URDF with xacro
    panda2_doc = xacro.process_file(
        urdf_xacro_path,
        mappings={
            'namespace': 'panda2',
            'controller_config': controllers_file
        }
    )
    panda2_root = panda2_doc.getroot()
    
    # Prefix Panda 1 elements
    prefix_urdf_elements(panda1_root, 'panda1_')
    
    # Prefix Panda 2 elements
    prefix_urdf_elements(panda2_root, 'panda2_')
    
    # Combine into unified URDF
    unified_root = combine_urdfs(panda1_root, panda2_root)
    
    # Convert to string
    rough_string = ET.tostring(unified_root, encoding='unicode')
    reparsed = minidom.parseString(rough_string)
    pretty = reparsed.toprettyxml(indent='  ')
    
    # Remove empty lines
    lines = [line for line in pretty.split('\n') if line.strip()]
    return '\n'.join(lines)


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: generate_unified_urdf.py <xacro_path> <controllers_file> [output_path]")
        sys.exit(1)
    
    xacro_path = sys.argv[1]
    controllers_file = sys.argv[2]
    
    unified = generate_unified_urdf_string(xacro_path, controllers_file)
    
    if len(sys.argv) >= 4:
        output_path = sys.argv[3]
        with open(output_path, 'w') as f:
            f.write(unified)
        print(f"Unified URDF written to {output_path}")
    else:
        print(unified)
