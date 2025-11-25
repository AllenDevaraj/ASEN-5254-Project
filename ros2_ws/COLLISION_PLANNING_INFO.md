# Collision-Free Planning Setup - What's Published and Why It Might Not Work

## Topics and States Published for MoveIt

### 1. Planning Scene Topic
**Topic:** `/monitored_planning_scene` (Type: `moveit_msgs/msg/PlanningScene`)
**Publisher:** `DualPandaUnifiedNode.planning_scene_pub`
**Contains:**
- All 6 objects (red_block, green_block, red_solid, green_solid, red_hollow, green_hollow)
- Table
- Hollow object held by Panda 2 (dynamic, updated when moved)
- Operation: `CollisionObject.MOVE` for dynamic objects, `CollisionObject.ADD` for static table

### 2. Joint States Topic
**Topic:** `/joint_states` (Type: `sensor_msgs/msg/JointState`)
**Publisher:** `joint_state_combiner` node
**Contains:**
- Unified joint states for BOTH arms (panda1_joint1 through panda1_joint7, panda2_joint1 through panda2_joint7)
- Mimic joints (panda1_finger_joint2, panda2_finger_joint2)
- **CRITICAL:** MoveIt uses this to know the current positions of BOTH arms for collision checking

### 3. Object Poses
**Source:** Gazebo via `ros_gz_bridge`
**Topics:** 
- `/model/<object_name>/pose` (for each of the 6 objects)
- Bridged to ROS2 and subscribed in `_sim_pose_callback`
- Stored in `self.objects` dictionary with live positions

## Why Collision-Free Planning Might Not Work

### Issue 1: Planning Scene Updates
- Objects are published with `CollisionObject.MOVE` operation (good for dynamic updates)
- BUT: Objects need to be published BEFORE each planning request
- Current code calls `_setup_planning_scene()` but timing might be off

### Issue 2: Joint State Inclusion
- Planning requests (`GetCartesianPath`, `GetPositionIK`) include `request.start_state.joint_state`
- This should include BOTH arms' joint states
- Current code: ✅ Includes unified joint_state in planning requests

### Issue 3: Planning Scene Frame
- Objects published in `world` frame ✅
- Planning requests use robot base frame (panda1_link0 or panda2_link0)
- MoveIt should handle frame transforms automatically ✅

### Issue 4: Held Objects
- Hollow object held by Panda 2 is added as collision object
- BUT: Attached objects should use `AttachedCollisionObject` not just `CollisionObject`
- Current code only uses `CollisionObject` which might not work correctly

### Issue 5: Planning Scene Topic
- Publishing to `/monitored_planning_scene` ✅
- But MoveIt's `move_group` might need objects published to `/planning_scene` as well
- Or might need both topics

## What to Check

1. **Verify Topics Are Active:**
   ```bash
   ros2 topic list | grep -E "planning_scene|joint_states"
   ros2 topic echo /monitored_planning_scene --once
   ros2 topic echo /joint_states --once
   ```

2. **Check MoveIt Receives Planning Scene:**
   - MoveIt's move_group should subscribe to `/monitored_planning_scene`
   - Verify with: `ros2 topic info /monitored_planning_scene`

3. **Verify Collision Objects in Scene:**
   - Objects should be visible in RViz planning scene
   - Check that all 6 objects + table + held hollow appear

4. **Check Joint State:**
   - Unified joint_states should contain all joints from both arms
   - Verify: `ros2 topic echo /joint_states` shows panda1_* and panda2_* joints

