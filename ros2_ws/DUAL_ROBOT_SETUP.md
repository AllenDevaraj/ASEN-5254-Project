# 🤖🤖 Dual Panda Robot Setup

## ✅ What Was Created

**New launch file:** `dual_panda_demo.launch.py`

This spawns **TWO Panda robots** facing each other across the workbench!

---

## 🎯 Robot Positions

### **Robot 1 (panda1):**
- **Position:** (0.0, 0.6, 0.0) - LEFT side of table
- **Rotation:** -90° (facing RIGHT toward table center)
- **Namespace:** `/panda1`

### **Robot 2 (panda2):**
- **Position:** (0.0, -0.6, 0.0) - RIGHT side of table  
- **Rotation:** +90° (facing LEFT toward table center)
- **Namespace:** `/panda2`

### **Visual Layout:**
```
                    Y-axis
                      ↑
                      |
    panda1 →  -----[TABLE]----- ← panda2
    (0, 0.6)     (0.7, 0)     (0, -0.6)
      ↓              ↓            ↓
   Facing        Center       Facing
   RIGHT                       LEFT
   
   Both robots face each other across the table!
```

---

## 🚀 How to Run

### **Dual Robot Demo:**
```bash
source install/setup.bash
ros2 launch panda_ign_description dual_panda_demo.launch.py
```

### **Single Robot Demo (original):**
```bash
source install/setup.bash
ros2 launch panda_ign_description panda_ignition_demo.launch.py
```

---

## 🔍 What You'll See

**In Gazebo Harmonic:**
- 🤖 **Panda 1** on left side (y=0.6)
- 🤖 **Panda 2** on right side (y=-0.6)
- Both facing each other
- ⬛ Workbench in the middle
- 🟧 Orange peg (80×30×30mm)
- 🟦 Blue receptacle (80×50×50mm outer, 80×30.5×30.5mm inner)
- 🟥🟦🟩 Colored blocks
- 📷 Camera above

---

## 📡 ROS2 Topics (Dual Robots)

### **Robot 1:**
- `/panda1/joint_states`
- `/panda1/robot_description`
- `/panda1/panda_arm_controller/...`

### **Robot 2:**
- `/panda2/joint_states`
- `/panda2/robot_description`
- `/panda2/panda_arm_controller/...`

### **Shared:**
- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/clock`

---

## 🎮 Control Each Robot

### **Move Robot 1:**
```bash
ros2 topic pub --once /panda1/panda_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "..."
```

### **Move Robot 2:**
```bash
ros2 topic pub --once /panda2/panda_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "..."
```

---

## 🔥 Quick Launch

```bash
cd ~/ASEN-5254-Project/ros2_ws
source install/setup.bash
ros2 launch panda_ign_description dual_panda_demo.launch.py
```

**Two robots facing each other!** 🤖↔️🤖

