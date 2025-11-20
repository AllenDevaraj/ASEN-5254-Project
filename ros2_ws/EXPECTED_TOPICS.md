# Expected Topics When Simulation is Running

## Prerequisites
**The simulation MUST be running** for these topics to exist. Topics only appear when nodes are active.

## Expected Topics (When Dual Panda Simulation is Running)

### Panda1 Namespace Topics:
```
/panda1/joint_states
/panda1/panda_arm_controller/joint_trajectory
/panda1/panda_arm_controller/state
/panda1/panda_gripper_controller/gripper_cmd
/panda1/robot_description
/panda1/compute_ik                    # From MoveIt
/panda1/move_group/...                # Various MoveIt services
```

### Panda2 Namespace Topics:
```
/panda2/joint_states
/panda2/panda_arm_controller/joint_trajectory
/panda2/panda_arm_controller/state
/panda2/panda_gripper_controller/gripper_cmd
/panda2/robot_description
/panda2/compute_ik                    # From MoveIt
/panda2/move_group/...                # Various MoveIt services
```

### Shared Topics (No Namespace):
```
/clock                                # Simulation clock
/camera/color/image_raw               # Camera feed
/camera/depth/image_raw               # Depth camera
/parameter_events                     # ROS2 parameter events
/rosout                               # ROS2 logging
```

## How to Verify Namespace Isolation

### Step 1: Start the simulation
```bash
cd /home/the2xman/ASEN-5254-Project/ros2_ws
source install/setup.bash
ros2 launch pick_and_place dual_panda_complete.launch.py
```

### Step 2: In a NEW terminal, check topics
```bash
cd /home/the2xman/ASEN-5254-Project/ros2_ws
source install/setup.bash

# List all topics
ros2 topic list

# Check panda1 topics
ros2 topic list | grep "^/panda1/"

# Check panda2 topics
ros2 topic list | grep "^/panda2/"

# Check IK services
ros2 service list | grep compute_ik
```

### Step 3: Monitor a specific topic
```bash
# Terminal 1: Monitor panda1 joint trajectory
ros2 topic echo /panda1/panda_arm_controller/joint_trajectory

# Terminal 2: Monitor panda2 joint trajectory
ros2 topic echo /panda2/panda_arm_controller/joint_trajectory

# Terminal 3: Run panda1 GUI - should only publish to Terminal 1
ros2 run pick_and_place panda1_ik_gui

# Terminal 4: Run panda2 GUI - should only publish to Terminal 2
ros2 run pick_and_place panda2_ik_gui
```

## Troubleshooting

### If no topics appear:
1. **Is the simulation running?** Check with `ros2 node list`
2. **Are controllers loaded?** Check controller manager: `ros2 control list_controllers`
3. **Is MoveIt running?** Check for move_group nodes: `ros2 node list | grep move_group`

### If controllers fail to load:
The dual_panda_demo.launch.py needs to be updated to properly configure controllers for namespaced robots. Currently, it's trying to load controllers without namespaces, which causes failures.

### If GUIs show "No executable found":
```bash
cd /home/the2xman/ASEN-5254-Project/ros2_ws
colcon build --packages-select pick_and_place --symlink-install
source install/setup.bash
```

