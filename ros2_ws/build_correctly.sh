#!/bin/bash
set -e

echo "=========================================="
echo "Building ROS2 Workspace Correctly"
echo "=========================================="

# Make absolutely sure venv is NOT active
unset VIRTUAL_ENV
unset PYTHONHOME
export PATH=$(echo $PATH | tr ':' '\n' | grep -v amp5254 | tr '\n' ':')

# Use system Python explicitly
export PYTHON_EXECUTABLE=/usr/bin/python3

cd ~/ASEN-5254-Project/ros2_ws

echo "✓ Using system Python: $(which python3)"
echo "✓ Python version: $(python3 --version)"

# Check catkin_pkg
if ! python3 -c "import catkin_pkg" 2>/dev/null; then
    echo "Installing catkin_pkg..."
    /usr/bin/python3 -m pip install --user catkin_pkg
fi

echo "✓ catkin_pkg available"

# Source ROS2
source /opt/ros/humble/setup.bash

# Clean previous failed build
echo "Cleaning previous build..."
rm -rf build/panda_ign_description install/panda_ign_description

# Build
echo "Building workspace..."
colcon build --symlink-install \
    --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ Build Successful!"
    echo "=========================================="
    echo ""
    echo "Run the demo:"
    echo "  source install/setup.bash"
    echo "  ros2 launch panda_ign_description panda_ignition_demo.launch.py"
else
    echo "✗ Build failed!"
    exit 1
fi


