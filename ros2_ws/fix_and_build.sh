#!/bin/bash
set -e

echo "=========================================="
echo "Fix Build Issues and Build Workspace"
echo "=========================================="

# Deactivate venv if active
if [[ "$VIRTUAL_ENV" != "" ]]; then
    echo "Deactivating virtual environment..."
    deactivate
fi

cd ~/ASEN-5254-Project/ros2_ws

# Source ROS2
echo "Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

# Install catkin_pkg for system Python (needed for ament_cmake)
echo "Installing catkin_pkg for system Python..."
pip3 install --user catkin_pkg

# Clean previous build
echo "Cleaning previous failed build..."
rm -rf build/panda_ign_description install/panda_ign_description

# Build workspace (skip problematic franka packages)
echo "Building workspace..."
colcon build --symlink-install \
    --packages-skip franka_hardware franka_bringup franka_robot_state_broadcaster \
                    integration_launch_testing franka_gripper franka_example_controllers \
                    franka_gazebo_bringup franka_ros2 franka_fr3_moveit_config \
                    franka_ign_ros2_control franka_semantic_components

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ Build successful!"
    echo "=========================================="
    echo ""
    echo "Next: Run the demo with:"
    echo "  source install/setup.bash"
    echo "  ros2 launch panda_ign_description panda_ignition_demo.launch.py"
else
    echo ""
    echo "=========================================="
    echo "✗ Build failed. Check errors above."
    echo "=========================================="
    exit 1
fi

