# Unified MoveGroup Setup for Dual Panda Arms

This document explains the unified move_group setup and what changes are needed compared to the dual-namespace setup.

## Overview

The unified setup uses a **single MoveIt move_group** to control both Panda arms, instead of two separate namespaced move_groups. This allows MoveIt to plan for both arms simultaneously and better handle collisions between them.

## Key Differences

### Current Setup (Dual Namespace)
- Two separate move_groups: `/panda1/move_group` and `/panda2/move_group`
- Each robot has its own robot_description
- Services are namespaced: `/panda1/compute_ik`, `/panda2/compute_ik`
- Joint states from separate topics: `/panda1/joint_states`, `/panda2/joint_states`
- GUI uses separate service clients for each arm

### Unified Setup (Single MoveGroup)
- **One move_group**: `/move_group` (no namespace)
- **Unified robot_description** with both arms (prefixed: `panda1_link0`, `panda2_link0`, etc.)
- **Planning groups**: `panda1_arm` and `panda2_arm` (defined in `dual_panda.srdf`)
- Services: `/compute_ik`, `/compute_cartesian_path` (specify planning_group in request)
- **Combined joint states**: `/joint_states` (merged from both robots)
- GUI uses single service clients, specifies `group_name` parameter in requests

## Files Created

1. **`dual_panda.srdf`** - Defines `panda1_arm` and `panda2_arm` planning groups
2. **`dual_panda_single_group.launch.py`** - Launch file for unified setup
3. **`dual_arm_single_group_gui.py`** - GUI adapted for single move_group

## Remaining Work

### 1. Unified URDF
The biggest challenge is creating a unified URDF that combines both pandas with prefixed names. Currently, the robots are spawned separately in Gazebo with unprefixed link/joint names.

**Solution Options:**
- Option A: Modify the spawn process to use prefixed names (complex, requires changing Gazebo spawn)
- Option B: Create a unified robot_description for MoveIt only (links don't match Gazebo exactly)
- Option C: Use MoveIt's ability to load multiple robot descriptions (but this requires separate move_groups)

**Recommended:** Create a Python script or xacro macro that generates a unified URDF with prefixed names. The launch file should process this unified URDF and pass it to MoveIt.

### 2. Joint State Combining
A `joint_state_publisher` node needs to combine `/panda1/joint_states` and `/panda2/joint_states` into a single `/joint_states` topic with prefixed joint names (`panda1_joint1`, `panda2_joint1`, etc.).

### 3. Controller Remapping
The controllers need to publish joint commands with prefixed names. This may require:
- Modifying controller configuration to handle prefixed joints
- Or using a joint_command_bridge node to remap unprefixed commands to prefixed joints

### 4. GUI Adaptation
The GUI needs to:
- Use single service clients (no namespace)
- Add `group_name` parameter to IK/Cartesian requests:
  ```python
  request.group_name = 'panda1_arm'  # or 'panda2_arm'
  ```
- Use prefixed joint names when sending trajectories
- Subscribe to unified `/joint_states` instead of separate topics

## Testing Checklist

- [ ] Unified URDF loads correctly in MoveIt
- [ ] Both planning groups (`panda1_arm`, `panda2_arm`) are recognized
- [ ] Joint states are properly combined and prefixed
- [ ] IK service works for both planning groups
- [ ] Cartesian path planning works
- [ ] Controllers can execute trajectories with prefixed joint names
- [ ] GUI can control both arms via single move_group
- [ ] Collision checking works between arms (no manual collision objects needed!)

## Benefits of Unified Setup

1. **Better Collision Avoidance**: MoveIt automatically knows about both arms and can plan collision-free paths
2. **Coordinated Planning**: Can plan motions for both arms simultaneously
3. **Simpler Architecture**: One move_group instead of two
4. **Less Manual Work**: No need to manually publish collision objects for the other arm

## Drawbacks

1. **Complexity**: Requires unified URDF with prefixed names
2. **Joint State Remapping**: Need to combine/prefix joint states from separate controllers
3. **Migration Effort**: Existing code assumes namespaced services

