#!/bin/bash
set -e

echo "=========================================="
echo "CLEAN BUILD - No Venv Contamination"
echo "=========================================="

cd ~/ASEN-5254-Project/ros2_ws

# Kill any venv traces
unset VIRTUAL_ENV
unset PYTHONHOME
unset PYTHONPATH

# Verify system Python
echo "Python executable: $(which python3)"
if [[ "$(which python3)" == *"amp5254"* ]]; then
    echo "ERROR: Still using venv Python!"
    echo "Run: hash -r"
    echo "Then try again"
    exit 1
fi

echo "✓ Using system Python"

# Install catkin_pkg if needed
if ! /usr/bin/python3 -c "import catkin_pkg" 2>/dev/null; then
    echo "Installing catkin_pkg..."
    /usr/bin/python3 -m pip install --user catkin_pkg
fi

# Source ROS2
source /opt/ros/humble/setup.bash

# Build with packages we need, skip franka hardware
echo ""
echo "Building packages..."
colcon build --symlink-install \
    --packages-select \
        pick_and_place_msgs \
        panda_ign_description \
        pick_and_place \
        moveit_resources_panda_description \
        moveit_resources_panda_moveit_config \
    --cmake-args \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DPython3_EXECUTABLE=/usr/bin/python3

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓✓✓ BUILD SUCCESSFUL! ✓✓✓"
    echo "=========================================="
    echo ""
    echo "Next steps:"
    echo "  source install/setup.bash"
    echo "  ros2 launch panda_ign_description panda_ignition_demo.launch.py"
    echo ""
else
    echo ""
    echo "Build failed. Check errors above."
    exit 1
fi


