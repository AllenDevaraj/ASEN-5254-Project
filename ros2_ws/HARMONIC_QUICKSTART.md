# 🚀 Gazebo Harmonic 8.10.0 - Quick Start

## ✅ You Have Gazebo Harmonic 8.10.0 
**Perfect! Everything is now configured for your setup.**

---

## 📦 Your Packages

### **1. `panda_ign_description`** (NEW - Robot description)
This package has:
- ✅ Panda robot URDF with Gazebo Harmonic plugins
- ✅ World file (workbench, blocks, bins, camera)
- ✅ ros2_control configuration
- ✅ Launch file to spawn everything

**NO NODES** - This is just the robot description!

### **2. `pick_and_place`** (Already exists - Application)
This package has:
- ✅ Python nodes (object_detector, controller, state_machine)
- ✅ Custom messages
- ✅ Application logic

**These nodes will connect to the robot LATER**

### **3. `pick_and_place_msgs`** (Already exists - Messages)
- ✅ Custom ROS2 messages for detected objects

---

## 🎯 What Happens When You Build & Run

### **Step 1: Build**
```bash
cd ~/ASEN-5254-Project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

**Builds:** All 3 packages (msgs, description, app)

### **Step 2: Launch**
```bash
source install/setup.bash
ros2 launch panda_ign_description panda_ignition_demo.launch.py
```

**Opens:**
1. ✅ **Gazebo Harmonic GUI** - 3D simulator window
2. ✅ **Pick-and-place world** - Workbench, bins, blocks
3. ✅ **Panda robot spawns** - At position (0, 0, 0)
4. ✅ **Controllers start** - Robot becomes controllable
5. ✅ **Camera topics publish** - RGB + Depth data

**YES, you will SEE the Franka/Panda robot in Gazebo!** 🤖

---

## 🔍 What You'll Actually See

### **In Gazebo Window:**
```
┌─────────────────────────────────────┐
│  Gazebo Harmonic 8.10.0             │
├─────────────────────────────────────┤
│                                     │
│    [Panda Robot] ←── You'll see this!
│         │                           │
│         └─ Arm + Gripper           │
│                                     │
│    [Workbench]  [Colored Blocks]   │
│    [Bins: R G B]                   │
│                                     │
│    [Camera above]                   │
│                                     │
└─────────────────────────────────────┘
```

### **In Terminal:**
```
[INFO] [launch]: All log files can be found below /home...
[INFO] [gz_sim-1]: process started with pid [1234]
[INFO] [robot_state_publisher-2]: process started with pid [1235]
[INFO] [spawner-3]: Loading controller 'joint_state_broadcaster'
[INFO] [spawner-3]: Successfully loaded controller joint_state_broadcaster
[INFO] [spawner-4]: Loading controller 'panda_arm_controller'
[INFO] [spawner-4]: Successfully loaded controller panda_arm_controller
✅ ALL SYSTEMS GO!
```

---

## 💻 Complete Commands

### **First Time Setup:**

```bash
# 1. Install dependencies (one time)
cd ~/ASEN-5254-Project/ros2_ws
./install_ignition_deps.sh

# 2. Build workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 3. Source workspace
source install/setup.bash
```

### **Every Time You Want to Run:**

```bash
# Start here
cd ~/ASEN-5254-Project/ros2_ws

# Source everything
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch!
ros2 launch panda_ign_description panda_ignition_demo.launch.py
```

---

## 🧪 Test It Works

After launching, open **new terminals** and check:

### **Check Robot is There:**
```bash
source install/setup.bash
gz topic -l | grep panda
```
Should show topics like `/model/panda/...`

### **Check Controllers:**
```bash
ros2 control list_controllers
```
Should show:
- `joint_state_broadcaster` [active]
- `panda_arm_controller` [active]  
- `panda_gripper_controller` [active]

### **Check Topics:**
```bash
ros2 topic list
```
Should include:
- `/joint_states`
- `/camera/color/image_raw`
- `/camera/depth/image_raw`

### **Move the Robot:**
```bash
# Test joint movement
ros2 topic pub --once /panda_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'],
  points: [{positions: [0.5, -0.5, 0.0, -2.0, 0.0, 1.5, 0.5], time_from_start: {sec: 2}}]
}"
```

**You should see the robot arm move!** 🎉

---

## 📊 Status Check

| Component | Status | Where |
|-----------|--------|-------|
| Gazebo Harmonic | ✅ Installed (8.10.0) | System |
| ROS2 Humble | ✅ Installed | System |
| `panda_ign_description` | ✅ Created | src/ |
| `pick_and_place` | ✅ Exists | src/ |
| `pick_and_place_msgs` | ✅ Exists | src/ |
| **Deps installed** | ⏳ **YOU DO THIS** | - |
| **Workspace built** | ⏳ **YOU DO THIS** | - |
| **Nodes running** | ⏳ Later (Phase 2) | - |

---

## 🎯 What "Nodes Come Later" Means

**Right now:**
- ✅ Robot description package (no nodes needed)
- ✅ Gazebo simulation
- ✅ Controllers

**Phase 2 (later):**
- Start `object_detector` node (vision)
- Start `controller` node (motion planning)  
- Start `state_machine` node (logic)
- Connect everything with MoveIt2

**Phases are separate!** First get robot working in Gazebo, THEN add application nodes.

---

## 🔥 TL;DR - Just Run This

```bash
cd ~/ASEN-5254-Project/ros2_ws

# One time
./install_ignition_deps.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Every time
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch panda_ign_description panda_ignition_demo.launch.py
```

**Expected result:** Gazebo opens, you see the Panda robot + environment!

---

## 🐛 If Something Breaks

### **Can't find gz_ros2_control:**
```bash
sudo apt install ros-humble-gz-ros2-control
```

### **Build fails:**
```bash
# Check you have moveit_resources
ros2 pkg list | grep moveit_resources_panda

# If missing, install:
sudo apt install ros-humble-moveit-resources-panda-description
```

### **Gazebo won't start:**
```bash
# Kill stuck processes
killall -9 gz ruby

# Try again
gz sim --version  # Should show 8.10.0
```

---

## ✅ Success Criteria

You'll know it worked when:

1. ✅ Gazebo Harmonic window opens
2. ✅ You see a white/gray Panda robot
3. ✅ Workbench, blocks, and bins are visible
4. ✅ Terminal says "Successfully loaded controllers"
5. ✅ `ros2 control list_controllers` shows 3 active controllers

**When you see all this = Phase 1 COMPLETE!** 🎉

Then we add MoveIt2 + application nodes (Phase 2).

---

**START HERE:** `./install_ignition_deps.sh` 

Then: `colcon build && ros2 launch panda_ign_description panda_ignition_demo.launch.py`


