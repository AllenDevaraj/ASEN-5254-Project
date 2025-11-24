# Unified MoveGroup Setup - Current Limitation

## Issue Identified

When running `dual_panda_single_group.launch.py`, you'll see errors like:

```
Error: Link 'panda1_link0' declared as part of a chain in group 'panda1_arm' is not known to the URDF
Error: Link 'panda2_link0' declared as part of a chain in group 'panda2_arm' is not known to the URDF
Warning: Group 'panda1_arm' must have at least one valid joint
Warning: Group 'panda2_arm' must have at least one valid joint
```

## Root Cause

The **unified SRDF** (`dual_panda.srdf`) defines planning groups with **prefixed names**:
- `panda1_arm` → expects links like `panda1_link0`, `panda1_link1`, etc.
- `panda2_arm` → expects links like `panda2_link0`, `panda2_link1`, etc.

However, the **URDF** being loaded still has **unprefixed names**:
- `panda_link0`, `panda_link1`, etc. (single robot)

**MoveIt cannot find the links/joints that the SRDF references, so all planning groups fail to load.**

## Why This Happens

The unified setup requires:
1. ✅ Unified SRDF (created) - defines `panda1_arm` and `panda2_arm` groups
2. ❌ Unified URDF (MISSING) - must combine both pandas with prefixed names

The current launch file uses the same URDF as the dual-namespace setup (unprefixed), which is incompatible with the unified SRDF.

## Solution Options

### Option A: Generate Unified URDF (Recommended for Full Functionality)
Create a unified URDF that combines both pandas with prefixed names:
- Process `panda.urdf` twice with prefixes
- Combine into single robot_description
- Update launch file to use this unified URDF

This is complex but provides the correct unified setup.

### Option B: Use Dual-Namespace Setup (Current Working Approach)
Keep using `dual_panda_complete.launch.py` which works correctly:
- Two separate move_groups with namespaces
- Each robot has its own URDF
- All functionality works

### Option C: Temporary Workaround (Limited Functionality)
Create a modified SRDF that uses unprefixed names but still defines two groups. This won't work correctly because MoveIt needs to distinguish between the two arms in a single robot_description.

## Current Status

- ✅ Launch file structure created (follows same patterns as original)
- ✅ Unified SRDF created with prefixed groups
- ✅ GUI skeleton created showing required changes
- ❌ Unified URDF generation not yet implemented
- ❌ MoveGroup fails because URDF/SRDF mismatch

## Next Steps to Complete

1. **Generate Unified URDF**: Create script/xacro to combine both pandas with prefixes
2. **Update Launch File**: Use the unified URDF instead of single robot URDF
3. **Test**: Verify move_group loads both planning groups correctly
4. **Complete GUI**: Finish implementing unified GUI logic

**The structure is ready, but the unified URDF is the critical missing piece.**

