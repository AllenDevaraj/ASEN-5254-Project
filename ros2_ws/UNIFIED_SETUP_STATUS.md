# Unified MoveGroup Setup - Status and Comparison

## Answer to Your Question: "Are you using the same logic?"

**YES**, the unified setup (`dual_panda_single_group.launch.py`) uses **the same logic and patterns** as the original (`dual_panda_complete.launch.py`), just adapted for a single move_group. Here's the comparison:

### ✅ SAME Logic/Patterns:

1. **Simulation Launch**: 
   - Both include `dual_panda_demo.launch.py` identically
   - Same robot spawning, same controllers

2. **MoveItConfigsBuilder**:
   - Same builder pattern: `MoveItConfigsBuilder('moveit_resources_panda')`
   - Same parameters: `ros2_control_hardware_type: 'mock_components'`
   - Same config files: `kinematics.yaml`, `ompl`, `moveit_controllers.yaml`

3. **Bridge Setup**:
   - Same `bridge_objects_global` node for ground truth poses
   - Same topic remapping: `/objects_poses_sim`

4. **GUI Launch**:
   - Same `ExecuteProcess` pattern: `python3 -m pick_and_place.dual_arm_gui`
   - Same timing: TimerAction with 12s delay

5. **Launch Structure**:
   - Same argument declarations
   - Same TimerAction sequencing
   - Same conditional GUI launch

### 🔄 DIFFERENT (But Following Same Patterns):

1. **MoveIt Setup**:
   - **Original**: Two separate move_groups (`/panda1/move_group`, `/panda2/move_group`)
   - **Unified**: One move_group (`/move_group`) with planning groups `panda1_arm`, `panda2_arm`

2. **SRDF File**:
   - **Original**: Uses `panda.srdf` (single arm)
   - **Unified**: Uses `dual_panda.srdf` (defines both `panda1_arm` and `panda2_arm` groups)

3. **Service Clients** (in GUI):
   - **Original**: `/panda1/compute_ik`, `/panda2/compute_ik` (namespaced)
   - **Unified**: `/compute_ik` with `group_name: 'panda1_arm'` or `'panda2_arm'` parameter

### ⚠️ MISSING Components (Required for Full Functionality):

1. **Unified URDF**: 
   - Current URDF has unprefixed links (`panda_link0`)
   - Unified SRDF expects prefixed links (`panda1_link0`, `panda2_link0`)
   - **Need**: Generate unified URDF combining both pandas with prefixes

2. **Joint State Merger**:
   - Current: Separate `/panda1/joint_states`, `/panda2/joint_states`
   - Unified needs: Single `/joint_states` with prefixed joint names
   - **Need**: Custom node to merge and prefix joint states

3. **Controller Updates**:
   - Controllers expect unprefixed joint names
   - Unified setup would need prefixed joint names
   - **Need**: Either update controllers or add remapping layer

## Files Created:

1. ✅ `dual_panda.srdf` - SRDF with dual-arm planning groups (COMPLETE)
2. ✅ `dual_panda_single_group.launch.py` - Launch file structure (STRUCTURE COMPLETE, needs unified URDF)
3. ✅ `dual_arm_single_group_gui.py` - GUI skeleton showing key changes (STRUCTURE COMPLETE, needs implementation)
4. ✅ Documentation explaining the approach

## Next Steps to Complete:

1. **Generate Unified URDF**: Create script/xacro to combine both pandas with prefixed names
2. **Joint State Merger**: Create custom ROS2 node to merge `/panda1/joint_states` + `/panda2/joint_states` → `/joint_states` with prefixes
3. **Complete GUI**: Fill in TODO sections in `dual_arm_single_group_gui.py` using logic from `dual_arm_gui.py`
4. **Test**: Verify single move_group can plan for both arms

## Current Status:

- ✅ Launch file structure matches original patterns
- ✅ SRDF defines dual-arm groups correctly
- ⚠️ Missing unified URDF (critical blocker)
- ⚠️ Missing joint state merger (required for single move_group)
- ⚠️ GUI is skeleton only (needs implementation)

**Bottom Line**: Yes, same logic and patterns, just adapted for unified architecture. The structure is ready, but needs the unified URDF and joint state merger to actually work.

