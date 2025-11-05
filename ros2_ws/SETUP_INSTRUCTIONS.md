# Panda Gazebo Simulation - Setup Instructions

## Current Status

✅ **Completed:**
- Added insertion task objects (peg and receptacle) to the world
- Fixed URDF configuration to use proper simulation-ready files
- Updated dependencies in package.xml

❌ **Needs Action:**
- Install required ROS 2 packages (see below)

## Required Installation Steps

Run these commands in your terminal:

```bash
# 1. Install Franka description package (contains proper simulation URDFs with inertial properties)
sudo apt update
sudo apt install -y ros-humble-franka-description

# 2. Install ROS 2 control packages (for robot controller management)
sudo apt install -y ros-humble-controller-manager \
                    ros-humble-ros2-control \
                    ros-humble-ros2-controllers \
                    ros-humble-gz-ros2-control

# 3. Rebuild your workspace
cd ~/ASEN-5254-Project/ros2_ws
colcon build --packages-select panda_ign_description
source install/setup.bash

# 4. Launch the simulation
ros2 launch panda_ign_description panda_ignition_demo.launch.py
```

## What You'll See

Once launched successfully, you'll have:

1. **Franka FR3 Robot** - 7-DOF robotic arm with gripper at (0, 0, 0)
2. **Solid Peg** (Orange) - 80×50×50mm cuboid on workbench
3. **Hollow Receptacle** (Blue) - With 82×52×52mm internal cavity (2mm clearance)
4. **Colored blocks** - Red, blue, green blocks for sorting tasks
5. **RGB-D Camera** - Positioned above the workspace

## Insertion Task Details

- **Peg dimensions:** 80mm × 50mm × 50mm
- **Receptacle cavity:** 82mm × 52mm × 52mm (internal)
- **Clearance:** 2mm on each side (1mm on each face)
- **Receptacle walls:** 5mm thick
- **Location:** Both on workbench at y=-0.3m

## Troubleshooting

### If the robot doesn't appear:
- Check that `franka_description` is installed: `ros2 pkg list | grep franka_description`
- Check for errors in terminal output
- Verify controllers are loaded: `ros2 control list_controllers`

### If simulation crashes:
- Make sure all dependencies are installed
- Check that Gazebo Ignition/Fortress is properly installed
- Review terminal output for specific error messages

## Files Modified

1. `panda_ign_description/urdf/panda_sim.urdf.xacro` - NEW: Proper simulation URDF
2. `panda_ign_description/launch/panda_ignition_demo.launch.py` - Updated to use new URDF
3. `panda_ign_description/worlds/pick_and_place_ign.sdf` - Added insertion objects
4. `panda_ign_description/package.xml` - Updated dependencies

## Why the Previous Setup Failed

The original setup used `moveit_resources_panda_description` which provides URDFs for **motion planning visualization only**. These URDFs lack:
- Inertial properties (mass, inertia tensors)
- Proper collision geometries for physics
- Simulation-specific parameters

The `franka_description` package provides **simulation-ready** URDFs with all required properties for Gazebo physics simulation.

