# 📦 Where the Cuboids Are Defined

## 🎯 **THE MAIN FILE:**

**File:** `ros2_ws/src/panda_ign_description/worlds/pick_and_place_ign.sdf`

**Path:** `/home/the2xman/ASEN-5254-Project/ros2_ws/src/panda_ign_description/worlds/pick_and_place_ign.sdf`

---

## 📊 **What's in There:**

### **Colored Blocks (Cuboids):**

**Lines 174-280:** Four colored blocks for picking up

```xml
<!-- Red blocks (line ~174) -->
<model name="block_red_1">
  <pose>0.4 -0.22 0.225 0 0 0</pose>  ← Position (x, y, z)
  <collision>
    <geometry>
      <box>
        <size>0.05 0.05 0.05</size>  ← 5cm cube
      </box>
    </geometry>
  </collision>
  <visual>
    <material>
      <ambient>1 0 0 1</ambient>  ← RED color
      <diffuse>1 0 0 1</diffuse>
    </material>
  </visual>
</model>

<model name="block_red_2">...</model>
<model name="block_blue_1">...</model>
<model name="block_green_1">...</model>
```

### **Bins (Cuboids):**

**Lines 117-165:** Three colored bins for placing blocks

```xml
<!-- Red bin (line ~117) -->
<model name="bin_red">
  <pose>-0.5 -0.27 0.11 0 0 0</pose>
  <visual>
    <geometry>
      <box>
        <size>0.15 0.15 0.05</size>  ← 15cm x 15cm bin
      </box>
    </geometry>
    <material>
      <ambient>1 0 0 0.5</ambient>  ← RED (semi-transparent)
    </material>
  </visual>
</model>

<model name="bin_green">...</model>
<model name="bin_blue">...</model>
```

### **Workbench (Big Cuboid):**

**Lines 61-85:** The black table

```xml
<model name="workbench">
  <pose>0.7 0 0.1 0 0 0</pose>
  <geometry>
    <box>
      <size>1.0 3.0 0.2</size>  ← 1m x 3m table
    </box>
  </geometry>
  <material>
    <ambient>0.2 0.2 0.2 1</ambient>  ← Dark gray/black
  </material>
</model>
```

---

## 🎨 **How to Modify Cuboids:**

### **Change Block Size:**
```xml
<size>0.05 0.05 0.05</size>  ← Change to 0.08 0.08 0.08 for bigger blocks
```

### **Change Block Position:**
```xml
<pose>0.4 -0.22 0.225 0 0 0</pose>  ← (x, y, z, roll, pitch, yaw)
```

### **Change Colors:**
```xml
<ambient>1 0 0 1</ambient>  ← RED (R G B Alpha)
<ambient>0 1 0 1</ambient>  ← GREEN
<ambient>0 0 1 1</ambient>  ← BLUE
<ambient>1 1 0 1</ambient>  ← YELLOW
```

### **Add More Blocks:**

Copy a block model and rename it:
```xml
<model name="block_yellow_1">
  <pose>0.5 0.1 0.225 0 0 0</pose>
  <!-- ... rest same as other blocks -->
  <ambient>1 1 0 1</ambient>  ← Yellow
</model>
```

---

## 📍 **Current Cuboid Positions:**

| Object | Position (x, y, z) | Size | Color |
|--------|-------------------|------|-------|
| block_red_1 | (0.4, -0.22, 0.225) | 5cm cube | Red |
| block_red_2 | (0.4, 0.22, 0.225) | 5cm cube | Red |
| block_blue_1 | (0.6, 0.22, 0.225) | 5cm cube | Blue |
| block_green_1 | (0.45, 0.0, 0.225) | 5cm cube | Green |
| bin_red | (-0.5, -0.27, 0.11) | 15cm box | Red |
| bin_green | (-0.5, 0.0, 0.11) | 15cm box | Green |
| bin_blue | (-0.5, 0.27, 0.11) | 15cm box | Blue |
| workbench | (0.7, 0, 0.1) | 1m x 3m | Black |

---

## 🔧 **Quick Edit:**

```bash
# Edit the world file
nano ~/ASEN-5254-Project/ros2_ws/src/panda_ign_description/worlds/pick_and_place_ign.sdf

# After editing, rebuild
cd ~/ASEN-5254-Project/ros2_ws
colcon build --packages-select panda_ign_description

# Launch to see changes
source install/setup.bash
ros2 launch panda_ign_description panda_ignition_demo.launch.py
```

---

**SUMMARY:**
- **File:** `src/panda_ign_description/worlds/pick_and_place_ign.sdf`
- **Blocks:** Lines 174-280 (4 colored cubes)
- **Bins:** Lines 117-165 (3 colored boxes)
- **Table:** Lines 61-85 (big black cuboid)

