#!/bin/bash

# Installation script for ROS2 Humble + Ignition Gazebo Fortress dependencies
# Run this script to install all required system packages

set -e

echo "=========================================="
echo "Installing ROS2 + Gazebo Harmonic Dependencies"
echo "=========================================="

# Update package list
echo "Updating package lists..."
sudo apt update

# Install ROS2-Ignition bridge
echo "Installing ros-gz (ROS2-Ignition bridge)..."
sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image \
    ros-humble-ros-gzgarden

# Install Gazebo Harmonic (if not already installed)
echo "Installing Gazebo Harmonic (already installed: 8.10.0)..."
# Harmonic already installed, skipping
# sudo apt install -y gz-harmonic

# Install ros2_control for Ignition
echo "Installing ros2_control and gz_ros2_control..."
sudo apt install -y \
    ros-humble-gz-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-gripper-controllers \
    ros-humble-effort-controllers \
    ros-humble-position-controllers

# Install MoveIt2
echo "Installing MoveIt2..."
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-task-constructor-core

# Install additional ROS2 tools
echo "Installing additional ROS2 tools..."
sudo apt install -y \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins

# Install development tools
echo "Installing development tools..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

echo ""
echo "=========================================="
echo "✓ Installation Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Source ROS2: source /opt/ros/humble/setup.bash"
echo "2. Build workspace: cd ros2_ws && colcon build"
echo "3. Source workspace: source install/setup.bash"
echo ""

