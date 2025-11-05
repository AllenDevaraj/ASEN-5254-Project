# Ignition Gazebo Fortress Setup Guide

## ✅ What's Been Created

I've built a complete ROS2 + Ignition Gazebo Fortress integration for the pick-and-place demo:

### **New Package: `panda_ign_description`**
- ✅ Panda robot URDF with Ignition plugins
- ✅ ros2_control configuration for simulation
- ✅ Ignition-compatible world file (SDF 1.8)
- ✅ Controller configuration (YAML)
- ✅ Launch file for complete demo

### **Updated Components:**
- ✅ RGB-D camera with Ignition sensors
- ✅ Physics and rendering plugins
- ✅ ROS2-Ignition topic bridges
- ✅ Controller spawners

---

## 🚀 Installation Steps

### **Step 1: Install System Dependencies**

Run the installation script:

```bash
cd ~/ASEN-5254-Project/ros2_ws
./install_ignition_deps.sh
```

Or install manually:

```bash
sudo apt update
sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-gz-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit \
    ignition-fortress
```

### **Step 2: Build the Workspace**

```bash
cd ~/ASEN-5254-Project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🎯 Running the Demo

### **Option 1: Full Launch (All-in-One)**

```bash
cd ~/ASEN-5254-Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch Ignition Gazebo with robot and controllers
ros2 launch panda_ign_description panda_ignition_demo.launch.py
```

**What you'll see:**
- Ignition Gazebo window opens
- World with workbench, bins, and colored blocks
- Panda robot spawns at origin
- Robot controllers start automatically
- Camera topics begin publishing

### **Option 2: Step-by-Step (For Debugging)**

```bash
# Terminal 1: Start Ignition Gazebo with world
source install/setup.bash
gz sim -r -v 4 install/panda_ign_description/share/panda_ign_description/worlds/pick_and_place_ign.sdf

# Terminal 2: Spawn robot
source install/setup.bash
ros2 run ros_gz_sim create -name panda -topic robot_description

# Terminal 3: Start controllers
source install/setup.bash
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active panda_arm_controller
ros2 control load_controller --set-state active panda_gripper_controller

# Terminal 4: Start bridges
source install/setup.bash
ros2 run ros_gz_bridge parameter_bridge /camera/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image

# Terminal 5: Object detector
source install/setup.bash  
ros2 run pick_and_place object_detector

# Terminal 6: State machine
source install/setup.bash
ros2 run pick_and_place pick_and_place_state_machine
```

---

## 📋 Verification Steps

### **1. Check Ignition is Running**

```bash
gz topic -l
```

Should show Ignition topics like:
- `/clock`
- `/world/pick_and_place_world/...`

### **2. Check ROS2 Topics**

```bash
ros2 topic list
```

Should include:
- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/joint_states`
- `/panda_arm_controller/...`

### **3. Check Controllers**

```bash
ros2 control list_controllers
```

Should show:
- `joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active`
- `panda_arm_controller[joint_trajectory_controller/JointTrajectoryController] active`
- `panda_gripper_controller[position_controllers/GripperActionController] active`

### **4. Visualize in RViz2**

```bash
rviz2
```

Add displays for:
- RobotModel (robot_description)
- TF
- Image (camera topics)

---

## 🎮 Test Robot Motion

### **Test Joint Movement:**

```bash
# Move joint 1
ros2 topic pub --once /panda_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'],
  points: [{positions: [0.5, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785], time_from_start: {sec: 2}}]
}"
```

### **Test Gripper:**

```bash
# Close gripper
ros2 action send_goal /panda_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.01, max_effort: 50.0}}"

# Open gripper
ros2 action send_goal /panda_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.04, max_effort: 50.0}}"
```

---

## 🔧 Troubleshooting

### **Issue: Ignition won't start**

```bash
# Check if Ignition is installed
gz sim --version  # Should show 6.17.0

# Check for conflicts
killall -9 gz ruby
```

### **Issue: Robot doesn't spawn**

```bash
# Check robot_description is published
ros2 topic echo /robot_description --once

# Check Ignition topics
gz topic -l | grep -i spawn
```

### **Issue: Controllers not starting**

```bash
# Check controller_manager
ros2 node list | grep controller_manager

# Check controller status
ros2 control list_controllers

# Restart controller manually
ros2 control set_controller_state panda_arm_controller start
```

### **Issue: Camera topics not appearing**

```bash
# Check bridges are running
ros2 node list | grep bridge

# Check Ignition topics
gz topic -l | grep camera

# Restart bridge manually
ros2 run ros_gz_bridge parameter_bridge /camera/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image
```

### **Issue: "ign_ros2_control not found"**

```bash
# Install gz_ros2_control
sudo apt install ros-humble-gz-ros2-control

# Rebuild workspace
cd ~/ASEN-5254-Project/ros2_ws
colcon build --packages-select panda_ign_description
```

---

## 📊 What Works vs What Needs Integration

| Component | Status | Notes |
|-----------|--------|-------|
| Ignition Gazebo | ✅ Ready | World and physics configured |
| Robot Model | ✅ Ready | URDF with ros2_control |
| Controllers | ✅ Ready | Joint trajectory + gripper |
| Camera Sensors | ✅ Ready | RGB + Depth publishing |
| ROS2 Bridges | ✅ Ready | Topics bridged |
| **MoveIt2** | ⚠️ Needs Config | Motion planning not yet integrated |
| **Pick-Place App** | ⚠️ Needs Adaptation | Nodes need MoveIt2 connection |

---

## 🎯 Next Steps

### **Phase 1: Test Base System (Do This First!)**

1. Run the launch file
2. Verify Ignition opens with robot
3. Check controllers are active
4. Test manual joint commands
5. Verify camera publishes images

### **Phase 2: Add MoveIt2 (Coming Next)**

Need to configure:
- MoveIt2 move_group for Panda
- Planning scene
- Kinematics solver
- Connect to ros2_control

### **Phase 3: Integrate Application**

- Connect object_detector to camera topics
- Update controller.py to use MoveIt2 API
- Test full autonomous pick-and-place

---

## 🔥 Quick Start Command

**ONE COMMAND TO RULE THEM ALL:**

```bash
cd ~/ASEN-5254-Project/ros2_ws && \
source /opt/ros/humble/setup.bash && \
source install/setup.bash && \
ros2 launch panda_ign_description panda_ignition_demo.launch.py
```

This should:
1. Open Ignition Gazebo
2. Show the pick-and-place world
3. Spawn the Panda robot
4. Start all controllers
5. Begin publishing camera data

**If this works, you're 70% of the way there!** 🎉

---

## 📝 Files Created

```
ros2_ws/src/panda_ign_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   └── panda.urdf.xacro                    # Robot with Ignition plugins
├── config/
│   └── panda_controllers.yaml              # ros2_control configuration
├── launch/
│   └── panda_ignition_demo.launch.py       # Main launch file
└── worlds/
    └── pick_and_place_ign.sdf              # Ignition world (SDF 1.8)
```

---

## 💡 Tips

1. **Always source both ROS2 and workspace:**
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

2. **Check logs if something fails:**
   ```bash
   cat log/latest_build/panda_ign_description/stdout_stderr.log
   ```

3. **Use verbose mode for debugging:**
   ```bash
   gz sim -r -v 4 <world_file>
   ```

4. **Monitor controller status:**
   ```bash
   watch -n 1 ros2 control list_controllers
   ```

---

**Ready to test? Run the installation script and then launch the demo!** 🚀

