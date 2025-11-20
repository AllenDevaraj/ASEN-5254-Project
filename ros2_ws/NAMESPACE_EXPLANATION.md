# Understanding Namespaces in Dual Panda Setup

## How to Check Topics with Namespaces

### Quick Commands:

```bash
# List all topics
ros2 topic list

# Filter for panda1 topics
ros2 topic list | grep "^/panda1/"

# Filter for panda2 topics  
ros2 topic list | grep "^/panda2/"

# Check IK services
ros2 service list | grep compute_ik

# Check action servers
ros2 action list | grep gripper
```

### Or use the helper script:
```bash
cd /home/the2xman/ASEN-5254-Project/ros2_ws
source install/setup.bash
./check_namespaced_topics.sh
```

---

## Topic Name Differences

### **Without Namespace (Single Panda):**
- `/panda_arm_controller/joint_trajectory` - publishes joint commands
- `/compute_ik` - IK service
- `/panda_gripper_controller/gripper_cmd` - gripper action

### **With Namespace (Dual Pandas):**

**Panda 1:**
- `/panda1/panda_arm_controller/joint_trajectory` - publishes to panda1
- `/panda1/compute_ik` - IK service for panda1
- `/panda1/panda_gripper_controller/gripper_cmd` - gripper action for panda1

**Panda 2:**
- `/panda2/panda_arm_controller/joint_trajectory` - publishes to panda2
- `/panda2/compute_ik` - IK service for panda2
- `/panda2/panda_gripper_controller/gripper_cmd` - gripper action for panda2

---

## How the GUI Code Handles Namespaces

### 1. **Entry Point Functions** (`panda1_main` and `panda2_main`)

```python
def panda1_main() -> None:
    run_gui(namespace='panda1')  # Passes 'panda1' as namespace

def panda2_main() -> None:
    run_gui(namespace='panda2')  # Passes 'panda2' as namespace
```

### 2. **Namespace Helper Function** (`_ns_topic`)

Located in `gui_end_effector.py` lines 70-74:

```python
def _ns_topic(self, relative: str) -> str:
    relative = relative.lstrip('/')
    if self.namespace:
        return f'/{self.namespace}/{relative}'  # Adds namespace prefix
    return f'/{relative}'  # No namespace
```

**How it works:**
- Input: `'panda_arm_controller/joint_trajectory'`
- With namespace `'panda1'`: Returns `/panda1/panda_arm_controller/joint_trajectory`
- With namespace `'panda2'`: Returns `/panda2/panda_arm_controller/joint_trajectory`
- Without namespace: Returns `/panda_arm_controller/joint_trajectory`

### 3. **Topic/Service Creation** (Lines 50-67)

```python
# Joint trajectory publisher
self.joint_pub = self.create_publisher(
    JointTrajectory,
    self._ns_topic('panda_arm_controller/joint_trajectory'),  # Uses _ns_topic()
    10,
)

# IK service client
self.ik_client = self.create_client(
    GetPositionIK, 
    self._ns_topic('compute_ik')  # Uses _ns_topic()
)

# Gripper action client
self.gripper_client = ActionClient(
    self,
    GripperCommand,
    self._ns_topic('panda_gripper_controller/gripper_cmd'),  # Uses _ns_topic()
)
```

---

## Example: Topic Name Resolution

### For `panda1_ik_gui`:
1. `panda1_main()` is called
2. `run_gui(namespace='panda1')` is called
3. `PandaIKNode(namespace='panda1')` is created
4. When creating publisher:
   - `self._ns_topic('panda_arm_controller/joint_trajectory')` is called
   - Returns: `/panda1/panda_arm_controller/joint_trajectory`
5. Publisher subscribes to `/panda1/panda_arm_controller/joint_trajectory`

### For `panda2_ik_gui`:
1. `panda2_main()` is called
2. `run_gui(namespace='panda2')` is called
3. `PandaIKNode(namespace='panda2')` is created
4. When creating publisher:
   - `self._ns_topic('panda_arm_controller/joint_trajectory')` is called
   - Returns: `/panda2/panda_arm_controller/joint_trajectory`
5. Publisher subscribes to `/panda2/panda_arm_controller/joint_trajectory`

---

## Visual Flow Diagram

```
panda1_ik_gui
    ↓
namespace='panda1'
    ↓
_ns_topic('panda_arm_controller/joint_trajectory')
    ↓
'/panda1/panda_arm_controller/joint_trajectory'
    ↓
Publishes to panda1 robot only

panda2_ik_gui
    ↓
namespace='panda2'
    ↓
_ns_topic('panda_arm_controller/joint_trajectory')
    ↓
'/panda2/panda_arm_controller/joint_trajectory'
    ↓
Publishes to panda2 robot only
```

---

## Testing Namespace Isolation

### Check if topics are properly namespaced:

```bash
# Terminal 1: Check panda1 topics
ros2 topic echo /panda1/panda_arm_controller/joint_trajectory

# Terminal 2: Check panda2 topics  
ros2 topic echo /panda2/panda_arm_controller/joint_trajectory

# Terminal 3: Use panda1 GUI - should only see messages in Terminal 1
ros2 run pick_and_place panda1_ik_gui

# Terminal 4: Use panda2 GUI - should only see messages in Terminal 2
ros2 run pick_and_place panda2_ik_gui
```

---

## Key Code Locations

- **Namespace helper**: `gui_end_effector.py` lines 70-74 (`_ns_topic` method)
- **Entry points**: `gui_end_effector.py` lines 308-313 (`panda1_main`, `panda2_main`)
- **Topic creation**: `gui_end_effector.py` lines 50-67 (publishers/clients)
- **Setup.py entry points**: `setup.py` lines 43-44 (console_scripts)

