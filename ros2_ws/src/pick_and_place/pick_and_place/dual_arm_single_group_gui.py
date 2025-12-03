#!/usr/bin/env python3

"""
Tkinter GUI for unified dual Panda control with SINGLE MoveIt move_group.

KEY DIFFERENCES from dual_arm_gui.py:
1. Uses SINGLE move_group (no namespace) instead of two separate ones
2. Specifies planning_group ('panda1_arm' or 'panda2_arm') in MoveIt requests
3. Uses prefixed joint names (panda1_joint1, panda2_joint1, etc.)
4. Subscribes to unified /joint_states (combined from both robots)
5. No need to manually publish collision objects for the other arm (MoveIt knows both)

This is a skeleton showing the structural changes. Most logic from dual_arm_gui.py
can be reused with these modifications.
"""

import math
import threading
import tkinter as tk
from functools import partial
from tkinter import ttk, messagebox
from typing import List, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import GripperCommand, FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject, RobotTrajectory, Constraints, PositionConstraint, OrientationConstraint, MotionPlanRequest, MotionPlanResponse, PlanningOptions
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetCartesianPath, GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from pick_and_place.task_manager import TaskManager


class DualPandaUnifiedNode(Node):
    """
    ROS2 node for unified dual Panda control via single move_group.
    
    NOTE: This is a skeleton showing the structure. It will not work until:
    1. Unified URDF is generated (with prefixed links: panda1_link0, panda2_link0, etc.)
    2. MoveGroup successfully loads with both planning groups
    3. Joint state merger node combines joint states with prefixes
    
    The logic follows the same patterns as dual_arm_gui.py, just adapted for single move_group.
    """

    def __init__(self) -> None:
        super().__init__('dual_panda_unified_gui_node')
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Robot world poses (from launch file)
        self.robot_poses = {
            'panda1': {'x': 0.0, 'y': 0.15, 'z': 0.0, 'yaw': 0.0},
            'panda2': {'x': 1.4, 'y': -0.15, 'z': 0.0, 'yaw': math.pi}
        }
        
        # Joint names with prefixes (for unified robot_description)
        self.joint_names = {
            'panda1': [f'panda1_joint{i}' for i in range(1, 8)],
            'panda2': [f'panda2_joint{i}' for i in range(1, 8)]
        }
        
        # Planning group names for MoveIt
        self.planning_groups = {
            'panda1': 'panda1_arm',
            'panda2': 'panda2_arm'
        }
        
        self.gripper_length = 0.11
        self.neutral_pose: List[float] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        
        # Latest Joint States (from unified /joint_states)
        self.joint_state = None
        
        # Latest EE Poses (from FK)
        self.ee_poses = {'panda1': None, 'panda2': None}
        self.fk_futures = {'panda1': None, 'panda2': None}

        # ============================================================
        # KEY CHANGE: Single move_group services (no namespace)
        # ============================================================
        # Unified MoveIt service clients (under move_group node name)
        # Try with node name prefix first, fallback to root if needed
        self.ik_client = self.create_client(GetPositionIK, '/move_group/compute_ik')
        self.fk_client = self.create_client(GetPositionFK, '/move_group/compute_fk')
        self.cartesian_path_client = self.create_client(GetCartesianPath, '/move_group/compute_cartesian_path')
        
        # MoveIt Motion Planning service for OMPL planning (full motion planning with curved paths)
        # This uses OMPL planners (RRT, RRTConnect, etc.) to automatically find collision-free curved paths
        # Service interface: moveit_msgs/srv/GetMotionPlan
        self.motion_plan_client = self.create_client(
            GetMotionPlan,
            '/plan_kinematic_path'
        )
        # Alternative service path
        self.motion_plan_client_alt = self.create_client(
            GetMotionPlan,
            '/move_group/plan_kinematic_path'
        )
        
        # MoveIt ExecuteTrajectory action for executing planned trajectories through MoveIt
        # This allows MoveIt to monitor execution and handle collisions better
        # Try both possible action paths
        self.execute_trajectory_action = ActionClient(
            self,
            ExecuteTrajectory,
            '/move_group/execute_trajectory'
        )
        self.execute_trajectory_action_alt = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )
        
        # Alternative: also try root-level services if move_group path doesn't work
        self.ik_client_alt = self.create_client(GetPositionIK, '/compute_ik')
        self.fk_client_alt = self.create_client(GetPositionFK, '/compute_fk')
        self.cartesian_path_client_alt = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        
        # Action clients (still namespaced because controllers are separate)
        self.traj_client_panda1 = ActionClient(
            self,
            FollowJointTrajectory,
            '/panda1/panda_arm_controller/follow_joint_trajectory',
        )
        self.traj_client_panda2 = ActionClient(
            self,
            FollowJointTrajectory,
            '/panda2/panda_arm_controller/follow_joint_trajectory',
        )
        self.gripper_client_panda1 = ActionClient(
            self,
            GripperCommand,
            '/panda1/panda_gripper_controller/gripper_cmd',
        )
        self.gripper_client_panda2 = ActionClient(
            self,
            GripperCommand,
            '/panda2/panda_gripper_controller/gripper_cmd',
        )
        
        # Wait for services (MoveIt takes time to start up)
        self.get_logger().info('Waiting for unified MoveIt services (this may take 10-20 seconds)...')
        self.services_available = {
            'ik': False,
            'fk': False,
            'cartesian': False,
            'motion_plan': False
        }
        # Increase timeout - MoveIt needs time to load unified URDF/SRDF
        service_timeout = 30.0  # 30 seconds
        
        # Try primary service paths first (/move_group/compute_ik)
        if self.ik_client.wait_for_service(timeout_sec=service_timeout):
            self.services_available['ik'] = True
            self.get_logger().info('✓ Unified compute_ik service is available at /move_group/compute_ik.')
        elif self.ik_client_alt.wait_for_service(timeout_sec=5.0):
            # Fallback to root-level service
            self.ik_client = self.ik_client_alt
            self.services_available['ik'] = True
            self.get_logger().info('✓ Unified compute_ik service is available at /compute_ik.')
        else:
            self.get_logger().warn('✗ Unified compute_ik service not available at /move_group/compute_ik or /compute_ik.')
            
        if self.fk_client.wait_for_service(timeout_sec=service_timeout):
            self.services_available['fk'] = True
            self.get_logger().info('✓ Unified compute_fk service is available at /move_group/compute_fk.')
        elif self.fk_client_alt.wait_for_service(timeout_sec=5.0):
            # Fallback to root-level service
            self.fk_client = self.fk_client_alt
            self.services_available['fk'] = True
            self.get_logger().info('✓ Unified compute_fk service is available at /compute_fk.')
        else:
            self.get_logger().warn('✗ Unified compute_fk service not available at /move_group/compute_fk or /compute_fk.')
            
        if self.cartesian_path_client.wait_for_service(timeout_sec=service_timeout):
            self.services_available['cartesian'] = True
            self.get_logger().info('✓ Unified compute_cartesian_path service is available at /move_group/compute_cartesian_path.')
        elif self.cartesian_path_client_alt.wait_for_service(timeout_sec=5.0):
            # Fallback to root-level service
            self.cartesian_path_client = self.cartesian_path_client_alt
            self.services_available['cartesian'] = True
            self.get_logger().info('✓ Unified compute_cartesian_path service is available at /compute_cartesian_path.')
        else:
            self.get_logger().warn('✗ Unified compute_cartesian_path service not available at /move_group/compute_cartesian_path or /compute_cartesian_path.')
        
        # Check motion planning service (for OMPL planning)
        if self.motion_plan_client.wait_for_service(timeout_sec=service_timeout):
            self.services_available['motion_plan'] = True
            self.get_logger().info('✓ Unified motion planning service is available at /plan_kinematic_path.')
        elif self.motion_plan_client_alt.wait_for_service(timeout_sec=5.0):
            # Fallback to alternative service path
            self.motion_plan_client = self.motion_plan_client_alt
            self.services_available['motion_plan'] = True
            self.get_logger().info('✓ Unified motion planning service is available at /move_group/plan_kinematic_path.')
        else:
            self.get_logger().warn('✗ Unified motion planning service not available at /plan_kinematic_path or /move_group/plan_kinematic_path.')
        
        # Schedule periodic service checks in case they become available later
        self.service_check_timer = self.create_timer(5.0, self._check_services_periodically)
    
    def _check_services_periodically(self):
        """Periodically check if MoveIt services become available."""
        if not self.services_available['ik']:
            if self.ik_client.service_is_ready():
                self.services_available['ik'] = True
                self.get_logger().info('✓ Unified compute_ik service became available at /move_group/compute_ik!')
            elif self.ik_client_alt.service_is_ready():
                self.ik_client = self.ik_client_alt
                self.services_available['ik'] = True
                self.get_logger().info('✓ Unified compute_ik service became available at /compute_ik!')
        if not self.services_available['fk']:
            if self.fk_client.service_is_ready():
                self.services_available['fk'] = True
                self.get_logger().info('✓ Unified compute_fk service became available at /move_group/compute_fk!')
            elif self.fk_client_alt.service_is_ready():
                self.fk_client = self.fk_client_alt
                self.services_available['fk'] = True
                self.get_logger().info('✓ Unified compute_fk service became available at /compute_fk!')
        if not self.services_available['cartesian']:
            if self.cartesian_path_client.service_is_ready():
                self.services_available['cartesian'] = True
                self.get_logger().info('✓ Unified compute_cartesian_path service became available at /move_group/compute_cartesian_path!')
            elif self.cartesian_path_client_alt.service_is_ready():
                self.cartesian_path_client = self.cartesian_path_client_alt
                self.services_available['cartesian'] = True
                self.get_logger().info('✓ Unified compute_cartesian_path service became available at /compute_cartesian_path!')
        
        # Check motion planning service
        if not self.services_available['motion_plan']:
            if self.motion_plan_client.service_is_ready():
                self.services_available['motion_plan'] = True
                self.get_logger().info('✓ Unified motion planning service became available at /plan_kinematic_path!')
            elif self.motion_plan_client_alt.service_is_ready():
                self.motion_plan_client = self.motion_plan_client_alt
                self.services_available['motion_plan'] = True
                self.get_logger().info('✓ Unified motion planning service became available at /move_group/plan_kinematic_path!')
        
        # Cancel timer if all services are available
        all_available = all(self.services_available.values())
        if all_available:
            if self.service_check_timer:
                self.service_check_timer.cancel()
            self.get_logger().info('✓ All MoveIt services are now available!')
            # Trigger GUI update callback if it exists
            if hasattr(self, '_on_services_ready_callback') and self._on_services_ready_callback:
                try:
                    self._on_services_ready_callback()
                except Exception as e:
                    self.get_logger().error(f'Error in services ready callback: {e}')
            
        self.get_logger().info('Waiting for action servers...')
        # Action servers should be available (controllers are separate)
        self.gripper_client_panda1.wait_for_server(timeout_sec=5.0)
        self.gripper_client_panda2.wait_for_server(timeout_sec=5.0)
        self.traj_client_panda1.wait_for_server(timeout_sec=5.0)
        self.traj_client_panda2.wait_for_server(timeout_sec=5.0)
        
        # ============================================================
        # KEY CHANGE: Single planning scene publisher (no namespace)
        # ============================================================
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/monitored_planning_scene',
            10
        )
        
        # ============================================================
        # KEY CHANGE: Subscribe to unified /joint_states
        # ============================================================
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',  # Unified topic (combined from both robots)
            self._joint_state_callback,
            10
        )
        
        # CRITICAL: Subscribe to object poses EARLY (before any blocking operations)
        # This ensures the subscription is registered immediately when the node starts
        self.get_logger().info('Creating subscription to /objects_poses_sim (CRITICAL FOR LIVE POSES)...')
        self.pose_sub = self.create_subscription(
            TFMessage,
            '/objects_poses_sim',
            self._sim_pose_callback,
            10
        )
        self.get_logger().info('✓ Subscription to /objects_poses_sim created - this will register when node spins.')
        
        # Object and table definitions (same as original)
        self.table_pose = Pose()
        self.table_pose.position.x = 0.7
        self.table_pose.position.y = 0.0
        self.table_pose.position.z = 0.1
        self.table_pose.orientation.w = 1.0
        self.table_size = [1.0, 3.0, 0.2]
        self.table_height = 0.2
        
        self.objects = {
            'red_block': {'position': [0.4, -0.02, 0.225], 'size': [0.05, 0.05, 0.05], 'yaw': 0.0},
            'green_block': {'position': [0.45, 0.2, 0.225], 'size': [0.05, 0.05, 0.05], 'yaw': 0.0},
            'red_solid': {'position': [0.6, -0.2, 0.25], 'size': [0.025, 0.025, 0.08], 'yaw': 4.7124},  # 270 degrees from world file
            'green_solid': {'position': [0.6, 0.2, 0.25], 'size': [0.025, 0.025, 0.08], 'yaw': 4.7124},  # 270 degrees from world file
            'red_hollow': {'position': [0.9, -0.2, 0.231], 'size': [0.08, 0.05, 0.05], 'yaw': -1.5708},
            'green_hollow': {'position': [0.9, 0.2, 0.231], 'size': [0.08, 0.05, 0.05], 'yaw': -1.5708},
            'blue_block': {'position': [0.9, 0.2, 0.381], 'size': [0.05, 0.05, 0.05], 'yaw': 0.0}  # 7th block on top of green_hollow
        }
        
        self.objects_lock = threading.Lock()
        self.held_objects = {'panda1': None, 'panda2': None}
        self.collision_object_ids = {'panda1': set(), 'panda2': set()}  # Track per arm
        
        # ============================================================
        # FRAME-BASED INSERTION: Fixed geometric transforms
        # ============================================================
        # H - hollow block body frame (at geometric center)
        # S - solid block body frame (at geometric center)
        # I - insertion frame at the mouth of the hollow block
        # G_h - hollow gripper TCP frame when grasping hollow
        # G_s - solid gripper TCP frame when grasping solid
        
        # Hollow dimensions: [length_x=0.08, width_y=0.05, height_z=0.05]
        # Solid dimensions: [width_x=0.025, depth_y=0.025, height_z=0.08] (upright)
        
        # ^H T_I: Hollow body frame to insertion frame (at opening face)
        # The opening is on the face with normal pointing toward -X (toward Panda 1)
        # Insertion frame I is at the center of the opening face
        # Position: half length forward along -X axis from hollow center
        self.H_T_I_position = [-0.08/2, 0.0, 0.0]  # Half length forward (opening is on -X face)
        self.H_T_I_orientation = [0.0, 0.0, 0.0, 1.0]  # Identity (same orientation as H)
        
        # ^S T_Gs: Solid body frame to solid gripper TCP
        # Solid is upright (height_z=0.08), gripper grasps it from sides
        # When solid is held horizontally, its center aligns with gripper TCP
        # Offset: solid center is at gripper TCP (no offset needed)
        self.S_T_Gs_position = [0.0, 0.0, 0.0]  # Solid center = gripper TCP
        self.S_T_Gs_orientation = [0.0, 0.0, 0.0, 1.0]  # Identity (solid orientation = gripper orientation when held)
        
        # ^H T_Gh: Hollow body frame to hollow gripper TCP
        # Hollow is held by Panda 2, gripper grasps it from sides
        # Hollow center is at gripper TCP (no offset needed)
        self.H_T_Gh_position = [0.0, 0.0, 0.0]  # Hollow center = gripper TCP
        self.H_T_Gh_orientation = [0.0, 0.0, 0.0, 1.0]  # Identity
        
        # Pre-insertion gap (distance from solid to hollow opening)
        self.pre_insert_gap = 0.30  # 30cm standoff (increased from 25cm for better reachability)
        self.insert_depth = 0.20  # 20cm insertion depth
        
        # Initialize Task Manager for TAMP (Task and Motion Planning)
        self.task_manager = TaskManager(
            objects=self.objects,
            table_height=self.table_height,
            clearance_threshold=0.15  # 15cm threshold for "on" relationship detection (allows for some floating/settling)
        )
        self.get_logger().info('TaskManager initialized for TAMP planning')
        
        # Initialize planning scene
        self._setup_planning_scene()
        
        # NOTE: Pose subscription was already created above (before objects/planning scene setup)
    
    def _joint_state_callback(self, msg: JointState):
        """Handle unified joint_states (contains both panda1_* and panda2_* joints)."""
        self.joint_state = msg
        
        # Extract joint states for each arm and trigger FK requests
        # The unified joint_states should contain prefixed joint names
        panda1_joints_dict = {}
        panda2_joints_dict = {}
        
        for i, name in enumerate(msg.name):
            if name.startswith('panda1_'):
                panda1_joints_dict[name] = msg.position[i] if i < len(msg.position) else 0.0
            elif name.startswith('panda2_'):
                panda2_joints_dict[name] = msg.position[i] if i < len(msg.position) else 0.0
        
        # Trigger FK requests for both arms (CRITICAL for end-effector pose tracking)
        # In unified setup, FK needs FULL unified joint state (both arms)
        for arm in ['panda1', 'panda2']:
            # Check if we should trigger FK (not already running)
            if self.fk_futures[arm] is None or (self.fk_futures[arm] and self.fk_futures[arm].done()):
                # Pass the FULL unified joint state to FK (MoveIt needs both arms' joint states)
                self._trigger_fk(arm, msg)  # msg is the unified JointState with all joints
    
    def _sim_pose_callback(self, msg: TFMessage):
        """Update object poses from global simulation info."""
        if not hasattr(self, '_pose_callback_count'):
            self._pose_callback_count = 0
            self.get_logger().info('✓ First pose callback received! Subscription is working.')
        self._pose_callback_count += 1
        
        with self.objects_lock:
            updated_objects = []
            for transform in msg.transforms:
                obj_name = transform.child_frame_id
                
                if obj_name in self.objects:
                    # Update position
                    trans = transform.transform.translation
                    old_pos = self.objects[obj_name].get('position', [0, 0, 0])
                    new_pos = [trans.x, trans.y, trans.z]
                    self.objects[obj_name]['position'] = new_pos
                    updated_objects.append(obj_name)
                    
                    # Update Orientation (RPY)
                    q = [
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w
                    ]
                    
                    # Quat to RPY
                    sinr_cosp = 2 * (q[3] * q[0] + q[1] * q[2])
                    cosr_cosp = 1 - 2 * (q[0] * q[0] + q[1] * q[1])
                    roll = math.atan2(sinr_cosp, cosr_cosp)

                    sinp = 2 * (q[3] * q[1] - q[2] * q[0])
                    if abs(sinp) >= 1:
                        pitch = math.copysign(math.pi / 2, sinp)
                    else:
                        pitch = math.asin(sinp)

                    siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
                    cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
                    yaw = math.atan2(siny_cosp, cosy_cosp)
                    
                    self.objects[obj_name]['yaw'] = yaw
                    self.objects[obj_name]['rpy'] = [roll, pitch, yaw]
                    
                    # Log if position changed significantly (for debugging)
                    pos_diff = [abs(old_pos[i] - new_pos[i]) for i in range(3)]
                    max_diff = max(pos_diff) if pos_diff else 0
                    if max_diff > 0.001:
                        self.get_logger().debug(f'Updated {obj_name} pose: ({new_pos[0]:.4f}, {new_pos[1]:.4f}, {new_pos[2]:.4f}) [diff: {max_diff:.4f}m]')
            
            # Log callback activity every 50 calls
            if self._pose_callback_count % 50 == 0:
                self.get_logger().info(f'Pose callback active: {len(updated_objects)} objects updated (total callbacks: {self._pose_callback_count})')
    
    def _trigger_fk(self, arm: str, joint_state: JointState):
        """Call FK service asynchronously for unified MoveIt setup."""
        # Use unified FK client (with fallback)
        client = self.fk_client if self.fk_client.service_is_ready() else (self.fk_client_alt if self.fk_client_alt.service_is_ready() else None)
        if not client:
            return
        
        req = GetPositionFK.Request()
        # Use robot base frame for this arm
        base_frame = f'panda{1 if arm == "panda1" else 2}_link0'
        req.header.frame_id = base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.fk_link_names = [f'panda{1 if arm == "panda1" else 2}_link8']  # End-effector link (tip of arm chain)
        
        # CRITICAL: Use FULL unified joint state (contains both panda1_* and panda2_* joints)
        # MoveIt needs the complete robot state in unified setup
        req.robot_state.joint_state = joint_state
        
        future = client.call_async(req)
        future.add_done_callback(partial(self._fk_done_callback, arm=arm))
        self.fk_futures[arm] = future
    
    def _fk_done_callback(self, future, arm: str):
        """Handle FK response and transform to world frame."""
        try:
            resp = future.result()
            if resp.error_code.val == resp.error_code.SUCCESS:
                if resp.pose_stamped:
                    # Pose in robot base frame
                    local_pose = resp.pose_stamped[0].pose
                    
                    # Transform to World (Manual TF)
                    robot = self.robot_poses[arm]
                    
                    # Rotate Local Position by Robot Yaw
                    cos_yaw = math.cos(robot['yaw'])
                    sin_yaw = math.sin(robot['yaw'])
                    
                    lx = local_pose.position.x
                    ly = local_pose.position.y
                    lz = local_pose.position.z
                    
                    wx = lx * cos_yaw - ly * sin_yaw + robot['x']
                    wy = lx * sin_yaw + ly * cos_yaw + robot['y']
                    wz = lz + robot['z']
                    
                    # Rotate Orientation: Convert quat to RPY, add robot yaw
                    q = [local_pose.orientation.x, local_pose.orientation.y, local_pose.orientation.z, local_pose.orientation.w]
                    
                    sinr_cosp = 2 * (q[3] * q[0] + q[1] * q[2])
                    cosr_cosp = 1 - 2 * (q[0] * q[0] + q[1] * q[1])
                    l_roll = math.atan2(sinr_cosp, cosr_cosp)

                    sinp = 2 * (q[3] * q[1] - q[2] * q[0])
                    if abs(sinp) >= 1:
                        l_pitch = math.copysign(math.pi / 2, sinp)
                    else:
                        l_pitch = math.asin(sinp)

                    siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
                    cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
                    l_yaw = math.atan2(siny_cosp, cosy_cosp)
                    
                    # Add base yaw
                    w_yaw = l_yaw + robot['yaw']
                    w_roll = l_roll
                    w_pitch = l_pitch
                    
                    # Store in world frame [x, y, z, roll, pitch, yaw]
                    self.ee_poses[arm] = [wx, wy, wz, w_roll, w_pitch, w_yaw]
            else:
                self.get_logger().error(f'FK service returned error code {resp.error_code.val} for {arm}')
        except Exception as e:
            self.get_logger().error(f'FK callback exception for {arm}: {e}')
    
    def _setup_planning_scene(self, use_live_poses: bool = True):
        """
        Set up planning scene with table and objects using LIVE poses from Gazebo.
        Adapted for unified setup using 'world' frame.
        
        Args:
            use_live_poses: If True, use live poses from Gazebo. If False, use self.objects dictionary.
        """
        import time
        self.get_logger().info('Setting up planning scene with LIVE object poses...')
        
        # Get live object poses from Gazebo (via pose subscriptions)
        # This ensures collision objects are at their ACTUAL current positions, not stale dictionary values
        with self.objects_lock:
            objects_copy = {k: v.copy() for k, v in self.objects.items()}
        
        # CRITICAL: Refresh object poses from live data if available
        if use_live_poses:
            # Wait a bit for latest pose updates
            time.sleep(0.1)
            # The objects dictionary should already be updated by _sim_pose_callback, but we'll log it
            for obj_name in objects_copy.keys():
                obj_data = self.get_pose(obj_name)
                if obj_data:
                    objects_copy[obj_name] = obj_data
                    self.get_logger().debug(f'[COLLISION] Using live pose for {obj_name}: {obj_data["position"]}')
        
        # Create planning scene message (unified - single scene for both arms)
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = []
        scene.robot_state.is_diff = True
        
        # Add table (WORLD FRAME)
        # Use ADD operation for static table (will persist, but re-ADD each time to ensure it's there)
        table_obj = CollisionObject()
        table_obj.header.frame_id = 'world'
        table_obj.header.stamp = self.get_clock().now().to_msg()
        table_obj.id = 'table'
        table_obj.operation = CollisionObject.ADD  # Table is static, ADD ensures it exists
        
        table_primitive = SolidPrimitive()
        table_primitive.type = SolidPrimitive.BOX
        table_primitive.dimensions = self.table_size
        
        table_obj.primitives = [table_primitive]
        table_obj.primitive_poses = [self.table_pose]
        scene.world.collision_objects.append(table_obj)
        
        # Add all objects (WORLD FRAME) - Use LIVE poses from Gazebo
        # Use MOVE operation for dynamic updates to reflect current positions
        for obj_name, obj_data in objects_copy.items():
            obj = CollisionObject()
            obj.header.frame_id = 'world'
            obj.header.stamp = self.get_clock().now().to_msg()
            obj.id = obj_name
            # Use MOVE operation to update position dynamically (will ADD if doesn't exist)
            obj.operation = CollisionObject.MOVE  # MOVE allows updating positions of existing objects
            
            obj_primitive = SolidPrimitive()
            obj_primitive.type = SolidPrimitive.BOX
            obj_primitive.dimensions = obj_data['size']
            
            # Create world pose from LIVE/current object position
            world_pose = Pose()
            world_pose.position.x = obj_data['position'][0]
            world_pose.position.y = obj_data['position'][1]
            world_pose.position.z = obj_data['position'][2]
            
            # Set orientation from yaw (or full RPY if available)
            if 'rpy' in obj_data and obj_data['rpy']:
                qx, qy, qz, qw = _euler_to_quaternion(obj_data['rpy'][0], obj_data['rpy'][1], obj_data['rpy'][2])
            else:
                qx, qy, qz, qw = _euler_to_quaternion(0.0, 0.0, obj_data.get('yaw', 0.0))
            world_pose.orientation.x = qx
            world_pose.orientation.y = qy
            world_pose.orientation.z = qz
            world_pose.orientation.w = qw
            
            obj.primitives = [obj_primitive]
            obj.primitive_poses = [world_pose]
            scene.world.collision_objects.append(obj)
            self.get_logger().debug(f'[COLLISION] Added {obj_name} at ({world_pose.position.x:.3f}, {world_pose.position.y:.3f}, {world_pose.position.z:.3f})')
        
        # CRITICAL: Also include the other arm's current state in the planning scene
        # This ensures MoveIt knows about the other arm's position for collision checking
        if self.joint_state:
            # Set robot state in planning scene so MoveIt knows both arms' positions
            scene.robot_state.joint_state = self.joint_state
            scene.robot_state.is_diff = False  # Complete state
            self.get_logger().debug(f'[COLLISION] Added robot state to planning scene: {len(self.joint_state.name)} joints')
        
        # Publish scene (unified publisher) - publish multiple times to ensure MoveIt receives it
        for i in range(5):  # Increased from 3 to 5 for better reliability
            self.planning_scene_pub.publish(scene)
            time.sleep(0.15)  # Increased delay to ensure MoveIt processes each update
        self.get_logger().info(f'[COLLISION] Planning scene published (unified) - {len(scene.world.collision_objects)} collision objects (table + {len(scene.world.collision_objects) - 1} objects) + robot state')
        
        # CRITICAL: Wait longer for scene to be processed by MoveIt before planning
        # MoveIt needs time to process the planning scene and update its internal collision checker
        time.sleep(1.0)  # Increased from 0.8 to 1.0 seconds - ensures MoveIt has processed the scene
        
        return scene  # Return scene so it can be attached to planning requests
    
    def get_pose(self, object_name: str) -> dict:
        """Get the latest pose of an object (thread-safe)."""
        with self.objects_lock:
            if object_name in self.objects:
                obj = self.objects[object_name]
                return {
                    'position': obj.get('position', [0, 0, 0]),
                    'size': obj.get('size', [0.05, 0.05, 0.05]),
                    'yaw': obj.get('yaw', 0.0),
                    'rpy': obj.get('rpy', [0, 0, 0])
                }
        return None
    
    def get_ee_pose(self, arm: str) -> List[float]:
        """Get current EE pose (stub for now)."""
        return self.ee_poses.get(arm)
    
    def compute_ik(self, pose: Pose, arm: str = 'panda1', base_frame: str = None) -> List[float]:
        """
        Compute IK using unified move_group.
        EXACT COPY from dual_arm_gui.py - uses robot base frame if not specified.
        """
        # Default to robot base frame if not specified (EXACT COPY from original)
        if base_frame is None:
            base_frame = f'panda{1 if arm == "panda1" else 2}_link0'
        
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.planning_groups[arm]  # 'panda1_arm' or 'panda2_arm'
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=5.0).to_msg()

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = base_frame  # Use robot base frame (EXACT COPY)
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        request.ik_request.pose_stamped = pose_stamped
        
        # CRITICAL: Provide FULL unified joint state (both arms) so MoveIt can automatically
        # check collisions between both arms. This is the whole point of unified MoveIt!
        if self.joint_state:
            # Include ALL joints from unified joint_state (both panda1_* and panda2_* joints)
            request.ik_request.robot_state.joint_state = self.joint_state

        future = self.ik_client.call_async(request)
        if not self._wait_for_future(future, timeout_sec=5.0):
            raise RuntimeError('IK service timed out')
        response = future.result()

        if not response:
            raise RuntimeError('IK service call failed (no response).')
        if response.error_code.val != response.error_code.SUCCESS:
            # Error code meanings:
            # -31 = NO_IK_SOLUTION (pose unreachable, collision, or outside joint limits)
            # -1 = INVALID_GROUP_NAME
            # -2 = INVALID_LINK_NAME
            error_code = response.error_code.val
            error_msg = f'IK failed with error code {error_code}'
            if error_code == -31:
                error_msg += ' (NO_IK_SOLUTION: pose may be unreachable, in collision, or outside joint limits)'
            self.get_logger().error(f'{arm}: {error_msg}')
            # Log the target pose for debugging
            self.get_logger().error(f'{arm}: Target pose in {base_frame}: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}')
            raise RuntimeError(error_msg)

        joint_state = response.solution.joint_state
        joint_map = {name: position for name, position in zip(joint_state.name, joint_state.position)}

        try:
            return [joint_map[name] for name in self.joint_names[arm]]
        except KeyError as exc:
            raise RuntimeError(f'Missing joint in IK solution: {exc}') from exc
    
    def send_joint_trajectory(self, joint_positions: List[float], arm: str = 'panda1', seconds: float = 3.0) -> bool:
        """
        Send joint trajectory to specified arm and wait for completion.
        
        KEY CHANGE: Remaps prefixed joint names back to non-prefixed for hardware controllers.
        MoveIt uses prefixed names (panda1_joint1) but controllers expect non-prefixed (panda_joint1).
        """
        traj_client = self.traj_client_panda1 if arm == 'panda1' else self.traj_client_panda2
        
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        
        # REMAP: Controllers expect non-prefixed joint names (panda_joint1, not panda1_joint1)
        # MoveIt returns prefixed names (panda1_joint1, panda2_joint1), but hardware controllers expect panda_joint1
        # Strip the numeric prefix: panda1_joint1 -> panda_joint1, panda2_joint1 -> panda_joint1
        controller_joint_names = []
        for name in self.joint_names[arm]:
            # Replace panda1_ or panda2_ with panda_
            if name.startswith('panda1_'):
                controller_name = name.replace('panda1_', 'panda_', 1)
            elif name.startswith('panda2_'):
                controller_name = name.replace('panda2_', 'panda_', 1)
            else:
                controller_name = name
            controller_joint_names.append(controller_name)
        
        self.get_logger().debug(f'{arm}: Remapping joints {self.joint_names[arm]} -> {controller_joint_names}')
        traj.joint_names = controller_joint_names  # Non-prefixed names for hardware controllers

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
        traj.points.append(point)
        
        goal.trajectory = traj
        
        if not traj_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'Trajectory action server unavailable for {arm}.')
            return False

        future = traj_client.send_goal_async(goal)
        if not self._wait_for_future(future, timeout_sec=1.0):
            return False
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'Trajectory goal rejected for {arm}.')
            return False

        res_future = goal_handle.get_result_async()
        if not self._wait_for_future(res_future):
            return False
        result = res_future.result()
        
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
             self.get_logger().warn(f'Trajectory execution failed with error code: {result.result.error_code}')
             return False
             
        return True
    
    def send_gripper_goal(self, width: float, arm: str = 'panda1') -> None:
        """Send gripper command to specified arm."""
        gripper_client = self.gripper_client_panda1 if arm == 'panda1' else self.gripper_client_panda2
        
        goal = GripperCommand.Goal()
        goal.command.position = width
        goal.command.max_effort = 100.0

        if not gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'Gripper action server unavailable for {arm}.')
            return

        future = gripper_client.send_goal_async(goal)
        if not self._wait_for_future(future, timeout_sec=3.0):
            self.get_logger().error(f'Gripper goal timed out for {arm}.')
            return
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error(f'Failed to send gripper goal for {arm} (no goal handle).')
            return
        if not goal_handle.accepted:
            self.get_logger().warn(f'Gripper goal rejected for {arm}.')
            return

        result_future = goal_handle.get_result_async()
        self._wait_for_future(result_future, timeout_sec=5.0)

    def _compute_cartesian_path(self, waypoints: List[Pose], arm: str = 'panda1', return_full_trajectory: bool = False) -> List[float] | RobotTrajectory:
        """Compute cartesian path through waypoints.
        EXACT COPY from dual_arm_gui.py - uses robot base frame.
        """
        cartesian_client = self.cartesian_path_client
        
        request = GetCartesianPath.Request()
        # Use robot base frame (EXACT COPY from original)
        robot_base = f'panda{1 if arm == "panda1" else 2}_link0'
        request.header.frame_id = robot_base
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = self.planning_groups[arm]  # 'panda1_arm' or 'panda2_arm'
        request.waypoints = waypoints
        request.max_step = 0.01  # Small steps for smooth motion
        request.jump_threshold = 0.0  # No jumps allowed
        request.avoid_collisions = True  # CRITICAL: Enable collision checking - must not collide with obstacles
        
        # CRITICAL: Provide FULL unified joint state (both arms) so MoveIt can automatically
        # check collisions between both arms and objects. This is the whole point of unified MoveIt!
        if not self.joint_state:
            self.get_logger().error(f'[COLLISION ERROR] No joint state available for {arm}! MoveIt cannot check collisions!')
            raise RuntimeError(f'No joint state available - cannot plan collision-free path')
        
        # Include ALL joints from unified joint_state (both panda1_* and panda2_* joints)
        request.start_state.joint_state = self.joint_state
        # Also set the robot state flag to indicate this is a complete state (not a diff)
        request.start_state.is_diff = False  # Complete state for proper collision checking
        
        # CRITICAL: Verify joint state includes both arms
        joint_names = request.start_state.joint_state.name
        has_panda1 = any('panda1_' in name for name in joint_names)
        has_panda2 = any('panda2_' in name for name in joint_names)
        self.get_logger().debug(f'[COLLISION CHECK] Joint state includes: panda1={has_panda1}, panda2={has_panda2}, total_joints={len(joint_names)}')
        
        if not has_panda1 or not has_panda2:
            self.get_logger().warn(f'[COLLISION WARNING] Joint state missing one arm! panda1={has_panda1}, panda2={has_panda2}. Collision checking may be incomplete!')
        
        future = cartesian_client.call_async(request)
        if not self._wait_for_future(future, timeout_sec=5.0):
            self.get_logger().warn('Cartesian path service timed out.')
            return None
        response = future.result()
        
        if not response:
            raise RuntimeError('Cartesian path service call failed.')
        
        # CRITICAL: Reject ANY path that is not 100% feasible to prevent collisions
        # If a path is only partially feasible, it means it hit an obstacle mid-path, which WILL cause collisions
        # Only allow paths that are 99%+ feasible (to account for minor numerical rounding errors)
        if response.fraction < 0.99:
            raise RuntimeError(f'[COLLISION PREVENTED] Cartesian path hit obstacle: Only {response.fraction * 100:.1f}% feasible. Path rejected to prevent collision. Required: 99%+ (nearly 100%)')
        
        if response.fraction < 1.0:
            self.get_logger().warn(f'Cartesian path nearly complete: {response.fraction * 100:.1f}% feasible (acceptable, >99%, minor rounding)')
        
        self.get_logger().info(f'[COLLISION-FREE] Cartesian path validated: {response.fraction * 100:.1f}% feasible - path is collision-free')
        
        # Extract joint positions from trajectory
        if not response.solution.joint_trajectory.points:
            raise RuntimeError('No trajectory points in cartesian path response')
        
        # If caller wants full trajectory (for MoveIt execution), return it
        if return_full_trajectory:
            # Create RobotTrajectory message for MoveIt execution
            robot_trajectory = RobotTrajectory()
            robot_trajectory.joint_trajectory = response.solution.joint_trajectory
            return robot_trajectory
        
        # Return joint positions from last waypoint (for backward compatibility)
        # Need to map from unified names to arm-specific order
        last_point = response.solution.joint_trajectory.points[-1]
        joint_map = {name: pos for name, pos in zip(
            response.solution.joint_trajectory.joint_names,
            last_point.positions
        )}
        
        try:
            return [joint_map[name] for name in self.joint_names[arm]]
        except KeyError as e:
            # If unified response returns all joints, we filter. If it returns partial, we match.
            raise RuntimeError(f"Missing joint in Cartesian response: {e}")
    
    def _plan_with_ompl(self, target_pose: Pose, arm: str = 'panda1', planner_id: str = 'RRTConnect', planning_scene: PlanningScene = None, relax_orientation: bool = False) -> RobotTrajectory:
        """
        Plan a collision-free path using OMPL motion planning (RRT, RRTConnect, etc.).
        
        This uses OMPL planners which can automatically find curved paths around obstacles,
        unlike Cartesian path planning which only does straight-line segments.
        
        CRITICAL: Planning scene must be provided or set up beforehand so OMPL can see obstacles.
        
        Args:
            target_pose: Target end-effector pose in robot base frame
            arm: Arm to use ('panda1' or 'panda2')
            planner_id: OMPL planner to use (e.g., 'RRTConnect', 'RRT', 'PRM')
            planning_scene: Optional PlanningScene to attach to request (if None, uses published scene)
        
        Returns:
            RobotTrajectory with planned collision-free path
        
        Raises:
            RuntimeError: If planning fails
        """
        import time
        
        if not self.joint_state:
            raise RuntimeError(f'No joint state available for {arm} - cannot plan with OMPL')
        
        # CRITICAL: Ensure planning scene is set up with latest object poses BEFORE planning
        # This ensures OMPL can see all obstacles (table, objects, other arm)
        if planning_scene is None:
            self.get_logger().info(f'{arm}: Setting up planning scene before OMPL planning...')
            planning_scene = self._setup_planning_scene(use_live_poses=True)
            # Additional wait after scene setup to ensure MoveIt has processed it
            time.sleep(0.5)
        
        # Try primary service path first, fallback to alternatives
        motion_plan_client = self.motion_plan_client
        if not motion_plan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'Primary motion plan service not available, trying alternatives...')
            motion_plan_client = self.motion_plan_client_alt
            if not motion_plan_client.wait_for_service(timeout_sec=2.0):
                raise RuntimeError(f'MoveIt Motion Planning service not available - cannot use OMPL planning')
        
        robot_base = f'panda{1 if arm == "panda1" else 2}_link0'
        eef_link = f'panda{1 if arm == "panda1" else 2}_link8'
        
        # Create motion plan request
        request = GetMotionPlan.Request()
        
        # Set up motion plan request
        plan_request = MotionPlanRequest()
        plan_request.group_name = self.planning_groups[arm]  # 'panda1_arm' or 'panda2_arm'
        plan_request.num_planning_attempts = 10
        plan_request.allowed_planning_time = 5.0
        plan_request.planner_id = planner_id  # OMPL planner (RRTConnect, RRT, etc.)
        
        # Set start state (current joint positions) - CRITICAL for collision checking
        # CRITICAL: Verify unified joint state includes both arms before planning
        if not self.joint_state:
            raise RuntimeError(f'[OMPL] Unified joint state not available for {arm} - cannot plan with OMPL')
        
        # Verify both arms are in joint state
        panda1_joints = [name for name in self.joint_state.name if name.startswith('panda1_')]
        panda2_joints = [name for name in self.joint_state.name if name.startswith('panda2_')]
        self.get_logger().info(f'[OMPL] Unified joint state: {len(panda1_joints)} panda1 joints, {len(panda2_joints)} panda2 joints - OMPL will check collisions for both arms')
        
        if len(panda1_joints) == 0 or len(panda2_joints) == 0:
            raise RuntimeError(f'[OMPL] Joint state incomplete: panda1={len(panda1_joints)} joints, panda2={len(panda2_joints)} joints. Cannot check inter-arm collisions!')
        
        plan_request.start_state.joint_state = self.joint_state
        plan_request.start_state.is_diff = False  # Complete state for collision checking
        
        # CRITICAL: The planning scene has already been published to /monitored_planning_scene
        # MoveIt's PlanningSceneMonitor subscribes to this topic and maintains the scene
        # We've already waited 1.0+ seconds after publishing, so MoveIt should have processed it
        # The planning scene is automatically used by OMPL during planning through PlanningSceneMonitor
        self.get_logger().info(f'[COLLISION] Planning scene published with {len(planning_scene.world.collision_objects)} collision objects - MoveIt should see them via PlanningSceneMonitor')
        
        # Set goal constraints (pose constraint for target pose)
        goal_constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = robot_base
        pos_constraint.header.stamp = self.get_clock().now().to_msg()
        pos_constraint.link_name = eef_link
        pos_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005]))  # 5mm tolerance
        pos_constraint.constraint_region.primitive_poses.append(target_pose)  # Target position
        pos_constraint.weight = 1.0
        goal_constraints.position_constraints.append(pos_constraint)
        
        # Orientation constraint (relaxed for put_down operations)
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = robot_base
        orient_constraint.header.stamp = self.get_clock().now().to_msg()
        orient_constraint.link_name = eef_link
        orient_constraint.orientation = target_pose.orientation
        if relax_orientation:
            # Very relaxed tolerance for put_down operations (allows more flexibility)
            orient_constraint.absolute_x_axis_tolerance = 0.5  # ~28 degrees
            orient_constraint.absolute_y_axis_tolerance = 0.5
            orient_constraint.absolute_z_axis_tolerance = 0.5
        else:
            # Normal tolerance for pick operations
            orient_constraint.absolute_x_axis_tolerance = 0.1  # ~5.7 degrees
            orient_constraint.absolute_y_axis_tolerance = 0.1
            orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0
        goal_constraints.orientation_constraints.append(orient_constraint)
        
        plan_request.goal_constraints = [goal_constraints]
        
        # Set workspace bounds (optional, helps with planning)
        plan_request.workspace_parameters.header.frame_id = robot_base
        plan_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        plan_request.workspace_parameters.min_corner.x = -2.0
        plan_request.workspace_parameters.min_corner.y = -2.0
        plan_request.workspace_parameters.min_corner.z = -2.0
        plan_request.workspace_parameters.max_corner.x = 2.0
        plan_request.workspace_parameters.max_corner.y = 2.0
        plan_request.workspace_parameters.max_corner.z = 2.0
        
        # Attach motion plan request to service request
        # Note: planning_options is not needed in MoveIt2 - plan_only behavior is automatic
        request.motion_plan_request = plan_request
        
        # Call motion planning service
        self.get_logger().info(f'{arm}: Planning collision-free path using OMPL planner "{planner_id}" with {len(planning_scene.world.collision_objects)} collision objects...')
        future = motion_plan_client.call_async(request)
        
        if not self._wait_for_future(future, timeout_sec=10.0):
            raise RuntimeError(f'OMPL motion planning service call timed out for {arm}')
        
        response = future.result()
        if not response:
            raise RuntimeError(f'OMPL motion planning service call failed for {arm}')
        
        # Check error code
        error_code = response.motion_plan_response.error_code.val
        if error_code != 1:  # 1 = SUCCESS
            error_string = response.motion_plan_response.error_code.string if hasattr(response.motion_plan_response.error_code, 'string') else str(error_code)
            self.get_logger().error(f'[OMPL ERROR] Motion planning failed for {arm}:')
            self.get_logger().error(f'  Error code: {error_code}')
            self.get_logger().error(f'  Error string: {error_string}')
            self.get_logger().error(f'  Planning group: {plan_request.group_name}')
            self.get_logger().error(f'  Planner ID: {planner_id}')
            self.get_logger().error(f'  Joint state: {len(plan_request.start_state.joint_state.name)} joints')
            self.get_logger().error(f'  Planning scene objects: {len(planning_scene.world.collision_objects)}')
            raise RuntimeError(f'OMPL motion planning failed for {arm} with error code {error_code}: {error_string}')
        
        # Extract planned trajectory
        if not response.motion_plan_response.trajectory or not response.motion_plan_response.trajectory.joint_trajectory:
            raise RuntimeError(f'OMPL motion planning returned empty trajectory for {arm}')
        
        planned_trajectory = response.motion_plan_response.trajectory
        self.get_logger().info(f'{arm}: ✓ Successfully planned collision-free path using OMPL ({len(planned_trajectory.joint_trajectory.points)} waypoints)')
        
        return planned_trajectory

    def _wait_for_future(self, future, timeout_sec=None):
        """Wait for future to complete without spinning the node."""
        import time
        start_time = time.time()
        while not future.done():
            if timeout_sec is not None and (time.time() - start_time) > timeout_sec:
                return False
            time.sleep(0.01)
        return True
    
    def execute_trajectory_moveit(self, robot_trajectory: RobotTrajectory, arm: str = 'panda1') -> bool:
        """
        Execute a planned trajectory through MoveIt's ExecuteTrajectory action.
        
        This allows MoveIt to monitor execution, handle collisions, and validate the trajectory.
        This is safer than directly sending trajectories to hardware controllers.
        
        Args:
            robot_trajectory: RobotTrajectory message from MoveIt planning
            arm: Arm to use ('panda1' or 'panda2')
        
        Returns:
            True if execution succeeded, False otherwise
        """
        import time
        
        # Try primary action path first, fallback to alternative
        action_client = self.execute_trajectory_action
        if not action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(f'Primary ExecuteTrajectory action not available, trying alternative...')
            action_client = self.execute_trajectory_action_alt
            if not action_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().error(f'MoveIt ExecuteTrajectory action server not available at either path!')
                return False
        
        # CRITICAL: Filter trajectory to only include joints for this arm
        # MoveIt's ExecuteTrajectory needs trajectory with joints that match the controller
        # Controller config expects prefixed names (panda1_joint1, panda2_joint1)
        # Filter to only include this arm's joints
        arm_prefix = 'panda1_' if arm == 'panda1' else 'panda2_'
        
        filtered_trajectory = RobotTrajectory()
        filtered_trajectory.joint_trajectory = JointTrajectory()
        filtered_trajectory.joint_trajectory.header = robot_trajectory.joint_trajectory.header
        
        # Find indices of joints for this arm
        arm_joint_indices = []
        for i, name in enumerate(robot_trajectory.joint_trajectory.joint_names):
            if name.startswith(arm_prefix):
                filtered_trajectory.joint_trajectory.joint_names.append(name)
                arm_joint_indices.append(i)
        
        if len(filtered_trajectory.joint_trajectory.joint_names) == 0:
            self.get_logger().error(f'{arm}: No joints found for {arm} in trajectory! Available: {robot_trajectory.joint_trajectory.joint_names}')
            # Fallback to direct execution
            self.get_logger().warn(f'{arm}: Falling back to direct trajectory execution...')
            return self._execute_trajectory_direct(robot_trajectory, arm=arm)
        
        # Filter trajectory points to only include this arm's joints
        for point in robot_trajectory.joint_trajectory.points:
            new_point = JointTrajectoryPoint()
            new_point.time_from_start = point.time_from_start
            new_point.positions = [point.positions[i] for i in arm_joint_indices]
            if point.velocities and len(point.velocities) == len(point.positions):
                new_point.velocities = [point.velocities[i] for i in arm_joint_indices]
            if point.accelerations and len(point.accelerations) == len(point.positions):
                new_point.accelerations = [point.accelerations[i] for i in arm_joint_indices]
            filtered_trajectory.joint_trajectory.points.append(new_point)
        
        self.get_logger().debug(f'{arm}: Filtered trajectory: {len(robot_trajectory.joint_trajectory.joint_names)} -> {len(filtered_trajectory.joint_trajectory.joint_names)} joints for {arm}')
        
        # Create goal for ExecuteTrajectory action
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = filtered_trajectory
        
        # Send goal
        self.get_logger().info(f'{arm}: Executing trajectory through MoveIt ExecuteTrajectory action...')
        future = action_client.send_goal_async(goal)
        
        if not self._wait_for_future(future, timeout_sec=5.0):
            self.get_logger().error(f'{arm}: ExecuteTrajectory goal send timed out')
            return False
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn(f'{arm}: ExecuteTrajectory goal rejected, falling back to direct execution...')
            # Fallback to direct execution if MoveIt execution fails
            return self._execute_trajectory_direct(robot_trajectory, arm=arm)
        
        # Wait for execution to complete
        # Check goal status instead of result.error_code (ROS2 action structure)
        import time
        start_time = time.time()
        timeout = 30.0
        
        while goal_handle.status not in [4, 5, 6]:  # 4=SUCCEEDED, 5=ABORTED, 6=CANCELED
            if time.time() - start_time > timeout:
                self.get_logger().warn(f'{arm}: ExecuteTrajectory execution timed out (status: {goal_handle.status}), falling back to direct execution...')
                return self._execute_trajectory_direct(robot_trajectory, arm=arm)
            time.sleep(0.1)
        
        if goal_handle.status == 5:  # 5 = ABORTED
            self.get_logger().warn(f'{arm}: ExecuteTrajectory execution aborted, falling back to direct execution...')
            return self._execute_trajectory_direct(robot_trajectory, arm=arm)
        if goal_handle.status == 6:  # 6 = CANCELED
            self.get_logger().warn(f'{arm}: ExecuteTrajectory execution canceled, falling back to direct execution...')
            return self._execute_trajectory_direct(robot_trajectory, arm=arm)
        
        # Get result for any additional info
        result_future = goal_handle.get_result_async()
        if self._wait_for_future(result_future, timeout_sec=2.0):
            result = result_future.result()
            if result and hasattr(result, 'error_code'):
                # Some versions have error_code in result
                if hasattr(result.error_code, 'val') and result.error_code.val != 1:
                    self.get_logger().warn(f'{arm}: ExecuteTrajectory completed but error_code: {result.error_code.val}')
        
        self.get_logger().info(f'{arm}: ✓ Successfully executed trajectory through MoveIt ExecuteTrajectory!')
        return True
    
    def _execute_trajectory_direct(self, robot_trajectory: RobotTrajectory, arm: str = 'panda1') -> bool:
        """
        Fallback: Execute trajectory directly to hardware controllers.
        Used when MoveIt ExecuteTrajectory action fails or is unavailable.
        
        Args:
            robot_trajectory: RobotTrajectory message from MoveIt planning
            arm: Arm to use ('panda1' or 'panda2')
        
        Returns:
            True if execution succeeded, False otherwise
        """
        # Extract joint positions from last point and execute directly
        arm_prefix = 'panda1_' if arm == 'panda1' else 'panda2_'
        
        # Find last point and extract positions for this arm
        if not robot_trajectory.joint_trajectory.points:
            self.get_logger().error(f'{arm}: No trajectory points for direct execution')
            return False
        
        last_point = robot_trajectory.joint_trajectory.points[-1]
        joint_map = {name: pos for name, pos in zip(
            robot_trajectory.joint_trajectory.joint_names,
            last_point.positions
        )}
        
        # Extract joint positions for this arm
        arm_joint_positions = []
        for joint_name in self.joint_names[arm]:
            if joint_name in joint_map:
                arm_joint_positions.append(joint_map[joint_name])
            else:
                self.get_logger().error(f'{arm}: Joint {joint_name} not found in trajectory')
                return False
        
        # Execute directly using send_joint_trajectory (fallback)
        self.get_logger().info(f'{arm}: Executing trajectory directly to hardware controller (fallback)...')
        return self.send_joint_trajectory(arm_joint_positions, arm=arm, seconds=5.0)
    
    def plan_to_pose(self, target_pose: Pose, arm: str = 'panda1', add_waypoints: bool = True, refresh_scene: bool = True, use_ompl: bool = True) -> List[float]:
        """
        Plan a collision-free path to target pose using OMPL motion planning (default) or Cartesian path planning.
        
        OMPL planning (use_ompl=True): Uses RRT/RRTConnect/etc. to automatically find curved paths around obstacles.
        This is the recommended method as it can naturally curve around obstacles without manual waypoints.
        
        Cartesian planning (use_ompl=False): Plans straight-line segments between waypoints.
        Requires intermediate waypoints to guide the path around obstacles.
        
        Args:
            target_pose: Target pose in robot base frame
            arm: Arm to use ('panda1' or 'panda2')
            add_waypoints: If True and use_ompl=False, add intermediate waypoints to go around obstacles
            refresh_scene: If True, refresh planning scene before planning
            use_ompl: If True (default), use OMPL motion planning. If False, use Cartesian path planning.
        
        Returns:
            List of joint positions for the target pose (from planned trajectory)
        
        Raises:
            RuntimeError: If planning fails
        """
        import math
        import time
        
        # Get current EE pose in robot base frame using FK
        robot_base = f'panda{1 if arm == "panda1" else 2}_link0'
        eef_link = f'panda{1 if arm == "panda1" else 2}_link8'
        
        if not self.joint_state:
            raise RuntimeError(f'No joint state available for {arm} - cannot plan path')
        
        # CRITICAL: Ensure planning scene is up-to-date with LIVE poses before planning
        # NOTE: The caller should have already called _setup_planning_scene() and _add_held_object_as_collision()
        # We refresh here as a safety measure, but this should NOT remove held object collisions
        # because _setup_planning_scene() only adds/updates objects, it doesn't remove existing collision objects
        self.get_logger().info(f'{arm}: Ensuring planning scene is up-to-date with LIVE poses before path planning...')
        
        # CRITICAL: Set up planning scene with LIVE object poses from Gazebo
        # This ensures collision objects are at their ACTUAL current positions
        planning_scene = self._setup_planning_scene(use_live_poses=True)  # Returns scene for attachment to request
        
        # CRITICAL: Verify joint state includes both arms for collision checking
        # Without complete joint state, MoveIt cannot check collisions between arms!
        if self.joint_state:
            panda1_joints = [name for name in self.joint_state.name if name.startswith('panda1_')]
            panda2_joints = [name for name in self.joint_state.name if name.startswith('panda2_')]
            if len(panda1_joints) == 0 or len(panda2_joints) == 0:
                raise RuntimeError(f'[COLLISION ERROR] Joint state incomplete: panda1={len(panda1_joints)} joints, panda2={len(panda2_joints)} joints. MoveIt cannot check collisions between arms!')
            else:
                self.get_logger().info(f'{arm}: ✓ Joint state complete: {len(panda1_joints)} panda1 joints, {len(panda2_joints)} panda2 joints - collision checking enabled')
        else:
            raise RuntimeError(f'[COLLISION ERROR] No joint state available! MoveIt cannot check for collisions!')
        
        # Get current EE pose in robot base frame using FK
        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = robot_base
        fk_request.header.stamp = self.get_clock().now().to_msg()
        fk_request.fk_link_names = [eef_link]
        fk_request.robot_state.joint_state = self.joint_state
        
        fk_client = self.fk_client
        fk_future = fk_client.call_async(fk_request)
        if not self._wait_for_future(fk_future, timeout_sec=3.0):
            raise RuntimeError('FK service call timed out - cannot get current pose')
        
        fk_response = fk_future.result()
        if not fk_response or not fk_response.pose_stamped or len(fk_response.pose_stamped) == 0:
            raise RuntimeError('FK service call failed - cannot get current pose')
        
        current_pose_robot_frame = fk_response.pose_stamped[0].pose
        
        # Build waypoints: add intermediate waypoints to go around obstacles
        waypoints = [current_pose_robot_frame]
        
        if add_waypoints:
            # Calculate intermediate waypoints that intelligently go around obstacles
            current_x = current_pose_robot_frame.position.x
            current_y = current_pose_robot_frame.position.y
            current_z = current_pose_robot_frame.position.z
            target_x = target_pose.position.x
            target_y = target_pose.position.y
            target_z = target_pose.position.z
            
            # Calculate distance and direction
            dx = target_x - current_x
            dy = target_y - current_y
            dz = target_z - current_z
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Get other arm's EE position to avoid it
            other_arm = 'panda2' if arm == 'panda1' else 'panda1'
            other_ee_data = self.get_ee_pose(other_arm)
            
            # If path is long (>15cm), add intermediate waypoints
            if distance > 0.15:
                # CRITICAL: Ensure waypoints are always above the table to avoid collisions
                # Table is at Z=0.2m, so waypoints must be at least 0.25m above table (Z >= 0.45m)
                table_z = self.table_height  # 0.2m
                min_clearance_above_table = 0.25  # 25cm clearance above table
                min_waypoint_z = table_z + min_clearance_above_table  # Z >= 0.45m
                
                # Waypoint 1: Lift up to clear obstacles (table, objects)
                intermediate1 = Pose()
                intermediate1.position.x = current_x + dx * 0.25  # 25% of the way
                intermediate1.position.y = current_y + dy * 0.25
                # Lift up significantly, ensuring we clear the table
                # Use max of: (current/target + 15cm) OR (table + 25cm clearance)
                intermediate1.position.z = max(max(current_z, target_z) + 0.15, min_waypoint_z)
                intermediate1.orientation = current_pose_robot_frame.orientation
                waypoints.append(intermediate1)
                self.get_logger().info(f'{arm}: Added waypoint 1 (lift) at Z={intermediate1.position.z:.3f}m (table at Z={table_z:.3f}m, ensuring {min_clearance_above_table*100:.0f}cm clearance)')
                
                # Waypoint 2: Go around other arm if it's in the way
                if other_ee_data:
                    # Transform other arm's EE to this arm's robot frame
                    other_ee_world = Pose()
                    other_ee_world.position.x = other_ee_data[0]
                    other_ee_world.position.y = other_ee_data[1]
                    other_ee_world.position.z = other_ee_data[2]
                    other_ee_robot = self._transform_to_robot_frame(other_ee_world, arm)
                    other_x = other_ee_robot.position.x
                    other_y = other_ee_robot.position.y
                    other_z = other_ee_robot.position.z
                    
                    # Check if other arm is between current and target
                    path_mid_x = (current_x + target_x) / 2.0
                    path_mid_y = (current_y + target_y) / 2.0
                    dist_to_other = math.sqrt((path_mid_x - other_x)**2 + (path_mid_y - other_y)**2)
                    
                    if dist_to_other < 0.25:  # Other arm is within 25cm of path center
                        # CRITICAL: Add MULTIPLE waypoints to create a smooth arc around the other arm
                        # Cartesian path planning uses straight-line segments, so we need many waypoints
                        # to approximate a curved path around Panda 2
                        
                        # Determine which side of the path the other arm is on
                        other_dy = other_y - path_mid_y
                        
                        # Determine safe side to go around (opposite of where other arm is)
                        if other_dy > 0:
                            # Other arm is on positive Y side (right), go to negative Y side (left)
                            safe_y_offset = -0.40  # 40cm to the left
                            side_str = "left (other arm on right)"
                        else:
                            # Other arm is on negative Y side (left), go to positive Y side (right)
                            safe_y_offset = +0.40  # 40cm to the right
                            side_str = "right (other arm on left)"
                        
                        safe_z = max(max(current_z, target_z, other_z) + 0.15, min_waypoint_z)  # Above all + table clearance
                        
                        # Create a multi-waypoint path: current → lift → side → forward → approach → target
                        # Calculate Y position along the path at each waypoint
                        # We'll interpolate between current_y and target_y, then add offset for safe side
                        
                        # Waypoint 2: Move to the side (away from other arm) at 15% of path
                        intermediate2 = Pose()
                        intermediate2.position.x = current_x + dx * 0.15  # 15% forward
                        path_y_at_15 = current_y + dy * 0.15  # Y position on path at 15%
                        intermediate2.position.y = path_y_at_15 + safe_y_offset  # Move to safe side
                        intermediate2.position.z = safe_z
                        intermediate2.orientation = current_pose_robot_frame.orientation
                        waypoints.append(intermediate2)
                        self.get_logger().info(f'{arm}: Added waypoint 2 (move to {side_str}) at ({intermediate2.position.x:.3f}, {intermediate2.position.y:.3f}, {intermediate2.position.z:.3f})')
                        
                        # Waypoint 3: Continue forward while staying on safe side (30% of path)
                        intermediate3 = Pose()
                        intermediate3.position.x = current_x + dx * 0.30  # 30% forward
                        path_y_at_30 = current_y + dy * 0.30  # Y position on path at 30%
                        intermediate3.position.y = path_y_at_30 + safe_y_offset  # Stay on safe side
                        intermediate3.position.z = safe_z
                        intermediate3.orientation = current_pose_robot_frame.orientation
                        waypoints.append(intermediate3)
                        self.get_logger().info(f'{arm}: Added waypoint 3 (forward on {side_str}) at ({intermediate3.position.x:.3f}, {intermediate3.position.y:.3f}, {intermediate3.position.z:.3f})')
                        
                        # Waypoint 4: Move forward more while still on safe side (50% of path)
                        intermediate4 = Pose()
                        intermediate4.position.x = current_x + dx * 0.50  # 50% forward
                        path_y_at_50 = current_y + dy * 0.50  # Y position on path at 50%
                        intermediate4.position.y = path_y_at_50 + safe_y_offset  # Stay on safe side
                        intermediate4.position.z = safe_z
                        intermediate4.orientation = current_pose_robot_frame.orientation
                        waypoints.append(intermediate4)
                        self.get_logger().info(f'{arm}: Added waypoint 4 (midpoint on {side_str}) at ({intermediate4.position.x:.3f}, {intermediate4.position.y:.3f}, {intermediate4.position.z:.3f})')
                        
                        # Waypoint 5: Start approaching target while moving back toward path center (70% of path)
                        intermediate5 = Pose()
                        intermediate5.position.x = current_x + dx * 0.70  # 70% forward
                        path_y_at_70 = current_y + dy * 0.70  # Y position on path at 70%
                        intermediate5.position.y = path_y_at_70 + safe_y_offset * 0.5  # Move halfway back toward path
                        intermediate5.position.z = safe_z
                        intermediate5.orientation = current_pose_robot_frame.orientation
                        waypoints.append(intermediate5)
                        self.get_logger().info(f'{arm}: Added waypoint 5 (approaching target from {side_str}) at ({intermediate5.position.x:.3f}, {intermediate5.position.y:.3f}, {intermediate5.position.z:.3f})')
                        
                        # Waypoint 6: Final approach, move back to path center (85% of path)
                        intermediate6 = Pose()
                        intermediate6.position.x = current_x + dx * 0.85  # 85% forward
                        path_y_at_85 = current_y + dy * 0.85  # Y position on path at 85%
                        intermediate6.position.y = path_y_at_85 + safe_y_offset * 0.25  # Mostly back to path center
                        intermediate6.position.z = safe_z
                        intermediate6.orientation = current_pose_robot_frame.orientation
                        waypoints.append(intermediate6)
                        self.get_logger().info(f'{arm}: Added waypoint 6 (final approach) at ({intermediate6.position.x:.3f}, {intermediate6.position.y:.3f}, {intermediate6.position.z:.3f})')
                        
                        self.get_logger().info(f'{arm}: Created multi-waypoint path around {other_arm} ({len(waypoints)} waypoints total before target)')
                    else:
                        # Other arm not in the way, add midpoint waypoint
                        intermediate2 = Pose()
                        intermediate2.position.x = current_x + dx * 0.5
                        intermediate2.position.y = current_y + dy * 0.5
                        # Ensure waypoint is above table
                        intermediate2.position.z = max(max(current_z, target_z) + 0.12, min_waypoint_z)  # Above table
                        intermediate2.orientation = current_pose_robot_frame.orientation
                        waypoints.append(intermediate2)
                        self.get_logger().info(f'{arm}: Added waypoint 2 (midpoint) at ({intermediate2.position.x:.3f}, {intermediate2.position.y:.3f})')
                else:
                    # Can't get other arm position, add midpoint waypoint
                    intermediate2 = Pose()
                    intermediate2.position.x = current_x + dx * 0.5
                    intermediate2.position.y = current_y + dy * 0.5
                    intermediate2.position.z = max(current_z, target_z) + 0.12
                    intermediate2.orientation = current_pose_robot_frame.orientation
                    waypoints.append(intermediate2)
                    self.get_logger().info(f'{arm}: Added waypoint 2 (midpoint, no other arm data)')
        
        waypoints.append(target_pose)
        
        # Choose planning method: OMPL (curved paths) or Cartesian (straight-line segments)
        robot_trajectory = None
        planning_method_used = None
        
        if use_ompl:
            # Use OMPL motion planning - automatically finds curved paths around obstacles
            # OMPL should automatically plan around Panda 2's arm from the correct side:
            # - Green solid: Panda 2 on RIGHT → OMPL goes around from LEFT
            # - Red solid: Panda 2 on LEFT → OMPL goes around from RIGHT
            # OMPL uses the unified joint state and planning scene to see all obstacles
            try:
                self.get_logger().info(f'{arm}: Using OMPL motion planning (RRTConnect) for automatic curved path planning around obstacles...')
                # CRITICAL: Pass planning scene to OMPL so it sees all obstacles (table, objects, other arm)
                # OMPL will automatically find the best path around Panda 2's arm
                robot_trajectory = self._plan_with_ompl(target_pose, arm=arm, planner_id='RRTConnect', planning_scene=planning_scene)
                planning_method_used = "OMPL"
                self.get_logger().info(f'{arm}: ✓ OMPL successfully planned collision-free path around obstacles')
            except Exception as ompl_error:
                self.get_logger().warn(f'{arm}: OMPL planning failed: {ompl_error}. Falling back to Cartesian path planning with waypoints...')
                use_ompl = False  # Fall back to Cartesian planning with waypoints
        
        if not robot_trajectory:
            # Fallback: Use Cartesian path planning with waypoints
            self.get_logger().info(f'{arm}: Using Cartesian path planning through {len(waypoints)} waypoints...')
            robot_trajectory = self._compute_cartesian_path(waypoints, arm=arm, return_full_trajectory=True)
            planning_method_used = "Cartesian"
            
            if not robot_trajectory:
                raise RuntimeError(f'Failed to plan collision-free path to target pose for {arm}')
        
        # Execute trajectory through MoveIt (allows MoveIt to monitor and handle collisions)
        self.get_logger().info(f'{arm}: Executing planned trajectory through MoveIt ExecuteTrajectory action...')
        if not self.execute_trajectory_moveit(robot_trajectory, arm=arm):
            raise RuntimeError(f'Failed to execute trajectory through MoveIt for {arm}')
        
        # Return target joint positions (extract from last point for backward compatibility)
        last_point = robot_trajectory.joint_trajectory.points[-1]
        joint_map = {name: pos for name, pos in zip(
            robot_trajectory.joint_trajectory.joint_names,
            last_point.positions
        )}
        
        try:
            target_joints = [joint_map[name] for name in self.joint_names[arm]]
            self.get_logger().info(f'{arm}: ✓ Successfully planned and executed collision-free path using {planning_method_used}!')
            return target_joints
        except KeyError as e:
            raise RuntimeError(f"Missing joint in trajectory: {e}")
    
    def pick_object(self, object_name: str, arm: str = 'panda1') -> bool:
        """
        Pick an object using 5-step pipeline.
        Adapted for unified setup: uses 'world' frame and prefixed link names.
        """
        import time
        
        # CRITICAL: Set up planning scene BEFORE picking to ensure collision-free planning
        # This ensures MoveIt knows about the table, all objects, and both arms
        self.get_logger().info(f'{arm}: Setting up planning scene for collision-free pick operation...')
        self._setup_planning_scene()  # This now includes robot state (both arms) for collision checking
        time.sleep(0.5)  # Wait for planning scene to be processed by MoveIt (increased from 0.3)
        
        # CRITICAL FIX: Force pose refresh - wait for pose callback to update
        # Read pose multiple times with small delays to ensure we get the latest
        initial_pose = self.get_pose(object_name)
        time.sleep(0.1)  # Give callback time to update
        obj_data = self.get_pose(object_name)
        time.sleep(0.1)  # Another small delay
        obj_data = self.get_pose(object_name)  # Final read
        
        if not obj_data:
            self.get_logger().error(f'Unknown object: {object_name}')
            return False
        
        obj_world_pos = obj_data['position']
        obj_size = obj_data['size']
        obj_yaw = obj_data['yaw']
        
        # Log comparison with initial pose to detect if updates happened
        if initial_pose:
            pos_diff = [
                abs(initial_pose['position'][i] - obj_world_pos[i]) 
                for i in range(3)
            ]
            max_diff = max(pos_diff)
            pose_status = "UPDATED" if max_diff > 0.001 else "UNCHANGED"
            self.get_logger().info(f'{arm}: {object_name} pose {pose_status} (max diff: {max_diff:.4f}m)')
        else:
            self.get_logger().warn(f'{arm}: Could not get initial pose for {object_name}')
        
        # Debug logging for offset issue
        self.get_logger().info(f'{arm}: Picking {object_name} at world position: x={obj_world_pos[0]:.4f}, y={obj_world_pos[1]:.4f}, z={obj_world_pos[2]:.4f}, yaw={obj_yaw:.4f}')
        
        obj_center_z_world = obj_world_pos[2]
        obj_top_z_world = obj_center_z_world + obj_size[2] / 2
        
        try:
            
            # Check if this is a thin object (requires more precision)
            min_xy_dimension = min(obj_size[0], obj_size[1])
            is_thin_object = min_xy_dimension <= 0.03  # 3cm or less is thin
            if is_thin_object:
                self.get_logger().info(f'{arm}: Detected THIN object ({min_xy_dimension*100:.1f}cm) - using precision grasp mode')
            
            # Step 1: Open gripper FIRST (wider for thin objects to ensure they fit)
            self.get_logger().info(f'{arm}: Opening gripper...')
            gripper_open_width = 0.06 if is_thin_object else 0.04  # Open wider for thin objects
            self.send_gripper_goal(gripper_open_width, arm=arm)
            time.sleep(1.5)
            
            # Step 2: Pre-grasp pose (15cm above object top) - Transform to robot frame
            world_pre_grasp = Pose()
            world_pre_grasp.position.x = obj_world_pos[0]
            world_pre_grasp.position.y = obj_world_pos[1]
            world_pre_grasp.position.z = obj_top_z_world + 0.15
            
            # Transform to robot base frame (like original)
            pre_grasp_pose = self._transform_to_robot_frame(world_pre_grasp, arm)
            
            # Orientation: Look down (Roll=180), adjust yaw RELATIVE TO ROBOT (like original)
            # Target Local Yaw = obj_yaw - robot_yaw
            target_yaw_local = obj_yaw - self.robot_poses[arm]['yaw']
            if obj_size[1] > obj_size[0]:
                target_yaw_local += (math.pi / 2.0)
            # Add 45 deg offset to grasp along diagonal (avoids corner grasping)
            target_yaw_local += (math.pi / 4.0)
            # Add 90 deg offset for most objects (blocks, hollows, flat objects)
            # BUT skip this for upright solids (tall in Z) as they need different orientation
            is_upright_solid = obj_size[2] > max(obj_size[0], obj_size[1]) * 1.5 and 'solid' in object_name
            if not is_upright_solid:
                target_yaw_local += (math.pi / 2.0)  # 90 deg offset for non-upright-solids
            
            qx, qy, qz, qw = _euler_to_quaternion(math.pi, 0.0, target_yaw_local)
            pre_grasp_pose.orientation.x = qx
            pre_grasp_pose.orientation.y = qy
            pre_grasp_pose.orientation.z = qz
            pre_grasp_pose.orientation.w = qw
            
            # Move to pre-grasp pose using IK (robot base frame) - OLD LOGIC: Direct IK
            self.get_logger().info(f'{arm}: Moving to pre-grasp pose (15cm above object)...')
            pre_grasp_joints = self.compute_ik(pre_grasp_pose, arm=arm)  # Uses robot base frame by default
            self.send_joint_trajectory(pre_grasp_joints, arm=arm, seconds=3.0)
            
            # RE-READ POSE for Real-Time Precision - Force fresh read with delays
            time.sleep(0.2)  # Wait for callback to process latest Gazebo pose
            obj_data = self.get_pose(object_name)
            if not obj_data:
                raise RuntimeError(f'Failed to re-read pose for {object_name} before descent')
            
            # Read again after small delay to ensure we have the absolute latest
            time.sleep(0.1)
            obj_data = self.get_pose(object_name)
            obj_world_pos = obj_data['position']
            obj_center_z_world = obj_world_pos[2]
            obj_yaw = obj_data['yaw']
            
            # Debug logging after re-read
            self.get_logger().info(f'{arm}: Re-read {object_name} position BEFORE DESCENT: x={obj_world_pos[0]:.4f}, y={obj_world_pos[1]:.4f}, z={obj_world_pos[2]:.4f}')
            
            # Orientation: Adjust yaw RELATIVE TO ROBOT (like original - this fixes Panda 1 offset!)
            target_yaw_local = obj_yaw - self.robot_poses[arm]['yaw']
            if obj_size[1] > obj_size[0]:
                target_yaw_local += (math.pi / 2.0)
            # Add 45 deg offset to grasp along diagonal (avoids corner grasping)
            target_yaw_local += (math.pi / 4.0)
            # Add 90 deg offset for most objects (blocks, hollows, flat objects)
            # BUT skip this for upright solids (tall in Z) as they need different orientation
            is_upright_solid = obj_size[2] > max(obj_size[0], obj_size[1]) * 1.5 and 'solid' in object_name
            if not is_upright_solid:
                target_yaw_local += (math.pi / 2.0)  # 90 deg offset for non-upright-solids
            
            qx, qy, qz, qw = _euler_to_quaternion(math.pi, 0.0, target_yaw_local)
            
            # Step 3: Correction & Descent - Transform to robot base frame
            new_obj_top_z = obj_center_z_world + obj_size[2] / 2
            
            world_new_pre_grasp = Pose()
            world_new_pre_grasp.position.x = obj_world_pos[0]
            world_new_pre_grasp.position.y = obj_world_pos[1]
            world_new_pre_grasp.position.z = new_obj_top_z + 0.15
            
            # Transform to robot base frame (CRITICAL for correct picking)
            new_pre_grasp_pose = self._transform_to_robot_frame(world_new_pre_grasp, arm)
            new_pre_grasp_pose.orientation.x = qx
            new_pre_grasp_pose.orientation.y = qy
            new_pre_grasp_pose.orientation.z = qz
            new_pre_grasp_pose.orientation.w = qw

            # Calculate New Grasp Pose (At object center) - Transform to robot frame
            # CRITICAL FIX: Use ACTUAL object position instead of hardcoded table height
            # This fixes the issue where red_hollow (z=0.231) was hitting the table
            # Objects may not be exactly at table_height + obj_size[2]/2, so use their actual position
            obj_center_z_actual = obj_world_pos[2]  # Use actual object center Z from live pose
            
            world_center_pose = Pose()
            world_center_pose.position.x = obj_world_pos[0]
            world_center_pose.position.y = obj_world_pos[1]
            world_center_pose.position.z = obj_center_z_actual  # Use actual position, not assumed table height
            
            local_center = self._transform_to_robot_frame(world_center_pose, arm)
            
            # TCP Target = Center + Gripper Length
            grasp_pose = Pose()
            grasp_pose.position.x = local_center.position.x
            grasp_pose.position.y = local_center.position.y
            grasp_pose.position.z = local_center.position.z + self.gripper_length
            grasp_pose.orientation.x = qx
            grasp_pose.orientation.y = qy
            grasp_pose.orientation.z = qz
            grasp_pose.orientation.w = qw
            
            # CRITICAL: Use multi-waypoint collision-free descent (same logic as insertion)
            # Instead of direct straight-line descent, use intermediate waypoints to help MoveIt avoid obstacles
            self.get_logger().info(f'{arm}: Planning collision-free descent path with intermediate waypoints...')
            
            # Refresh planning scene before descent to ensure MoveIt knows about all obstacles
            self.get_logger().info(f'{arm}: Refreshing planning scene before descent for collision-free planning...')
            self._setup_planning_scene()  # Ensure table, all objects, and both arms are known
            time.sleep(0.5)  # Wait for planning scene to be processed (increased from 0.2)
            
            # Multi-waypoint descent: Pre-grasp → Intermediate waypoint (clear obstacles) → Grasp
            # This helps MoveIt find a collision-free path, especially around the table
            waypoint_intermediate = Pose()
            waypoint_intermediate.position.x = new_pre_grasp_pose.position.x
            waypoint_intermediate.position.y = new_pre_grasp_pose.position.y
            # Intermediate waypoint: slightly lower than pre-grasp but still above table
            intermediate_z = new_pre_grasp_pose.position.z - 0.05  # 5cm below pre-grasp (still 10cm above object)
            waypoint_intermediate.position.z = max(intermediate_z, grasp_pose.position.z + 0.05)  # Never below 5cm from grasp
            waypoint_intermediate.orientation = new_pre_grasp_pose.orientation
            
            # Multi-waypoint path: Pre-grasp → Intermediate → Grasp
            # MoveIt will plan collision-free paths between these waypoints
            descent_waypoints = [new_pre_grasp_pose, waypoint_intermediate, grasp_pose]  # Robot frame poses
            self.get_logger().info(f'{arm}: Computing collision-free Cartesian path through {len(descent_waypoints)} waypoints...')
            
            # Try Cartesian descent with fallback to simpler path for thin objects
            descent_success = False
            try:
                descent_trajectory = self._compute_cartesian_path(descent_waypoints, arm=arm, return_full_trajectory=True)
                if self.execute_trajectory_moveit(descent_trajectory, arm=arm):
                    descent_success = True
            except Exception as cart_err:
                self.get_logger().warn(f'{arm}: Cartesian descent failed: {cart_err}')
            
            if not descent_success:
                # Fallback: Try direct descent (no intermediate waypoint) for thin objects
                self.get_logger().info(f'{arm}: Trying simpler 2-waypoint descent path...')
                try:
                    simple_waypoints = [new_pre_grasp_pose, grasp_pose]
                    descent_trajectory = self._compute_cartesian_path(simple_waypoints, arm=arm, return_full_trajectory=True)
                    if self.execute_trajectory_moveit(descent_trajectory, arm=arm):
                        descent_success = True
                except Exception as simple_err:
                    self.get_logger().warn(f'{arm}: Simple descent also failed: {simple_err}')
            
            if not descent_success:
                # Last resort: Use IK directly (no collision checking, but at least attempt grasp)
                self.get_logger().warn(f'{arm}: Using IK fallback for descent (no collision checking)...')
                grasp_joints = self.compute_ik(grasp_pose, arm=arm)
                if grasp_joints:
                    self.send_joint_trajectory(grasp_joints, arm=arm, seconds=2.0)
                    time.sleep(2.5)
                else:
                    raise RuntimeError(f'{arm}: All descent methods failed - cannot reach grasp pose')
            
            # Step 4: Grasp (close gripper symmetrically and attach object)
            # GripperCommand action handles mimic joints automatically to ensure symmetric closing
            self.get_logger().info(f'{arm}: Closing gripper symmetrically to grasp object...')
            self.send_gripper_goal(0.01, arm=arm)  # GripperCommand automatically ensures both fingers close equally
            time.sleep(1.5)  # Wait for gripper to fully close and ensure object is centered
            
            # Verify gripper is closed symmetrically (check mimic joint positions match)
            if self.joint_state:
                try:
                    finger1_name = f'panda{1 if arm == "panda1" else 2}_finger_joint1'
                    finger2_name = f'panda{1 if arm == "panda1" else 2}_finger_joint2'
                    finger1_idx = self.joint_state.name.index(finger1_name)
                    finger2_idx = self.joint_state.name.index(finger2_name)
                    finger1_pos = self.joint_state.position[finger1_idx]
                    finger2_pos = self.joint_state.position[finger2_idx]
                    finger_diff = abs(finger1_pos - finger2_pos)
                    if finger_diff > 0.001:  # 1mm tolerance
                        self.get_logger().warn(f'{arm}: Gripper fingers not symmetric! Diff: {finger_diff:.4f}m (finger1: {finger1_pos:.4f}, finger2: {finger2_pos:.4f})')
                    else:
                        self.get_logger().info(f'{arm}: Gripper closed symmetrically (both fingers: {finger1_pos:.4f}m)')
                except (ValueError, IndexError) as e:
                    self.get_logger().debug(f'{arm}: Could not verify gripper symmetry: {e}')
            
            # Attach object to gripper (planning scene) - UNIFIED PUBLISHER
            scene = PlanningScene()
            scene.is_diff = True
            
            attached_obj = AttachedCollisionObject()
            attached_obj.object.id = object_name
            attached_obj.object.operation = CollisionObject.REMOVE
            eef_link = f'panda{1 if arm == "panda1" else 2}_hand'  # Prefixed link name
            attached_obj.link_name = eef_link
            attached_obj.touch_links = [eef_link, f'panda{1 if arm == "panda1" else 2}_leftfinger', f'panda{1 if arm == "panda1" else 2}_rightfinger']
            
            scene.robot_state.attached_collision_objects = [attached_obj]
            # Include full unified joint state so MoveIt knows both arms' positions
            if self.joint_state:
                scene.robot_state.joint_state = self.joint_state
                scene.robot_state.is_diff = True
            self.planning_scene_pub.publish(scene)
            time.sleep(0.5)
            
            # Step 5: Multi-waypoint lift to avoid obstacles (especially other arm)
            # Strategy: Lift slightly → Move sideways (away from other arm) → Lift higher
            self.get_logger().info(f'{arm}: Lifting object using obstacle-avoiding waypoints...')
            
            # Determine safe direction to move (away from other arm)
            # Panda 1 is at x=0, y=+0.15, Panda 2 is at x=1.4, y=-0.15
            # So for Panda 1, move in +Y direction (away from center, away from Panda 2)
            # For Panda 2, move in -Y direction (away from center, away from Panda 1)
            if arm == 'panda1':
                lateral_offset = 0.15  # Move 15cm in +Y direction (away from Panda 2)
            else:
                lateral_offset = -0.15  # Move 15cm in -Y direction (away from Panda 1)
            
            # Waypoint 1: Small initial lift (5cm) - just enough to clear table
            waypoint1 = Pose()
            waypoint1.position.x = grasp_pose.position.x
            waypoint1.position.y = grasp_pose.position.y
            waypoint1.position.z = grasp_pose.position.z + 0.05
            waypoint1.orientation = grasp_pose.orientation
            
            # Waypoint 2: Move sideways while maintaining height (away from other arm)
            waypoint2 = Pose()
            waypoint2.position.x = grasp_pose.position.x
            waypoint2.position.y = grasp_pose.position.y + lateral_offset
            waypoint2.position.z = grasp_pose.position.z + 0.05
            waypoint2.orientation = grasp_pose.orientation
            
            # Waypoint 3: Lift higher (15cm total) now that we're clear of obstacles
            waypoint3 = Pose()
            waypoint3.position.x = grasp_pose.position.x
            waypoint3.position.y = grasp_pose.position.y + lateral_offset
            waypoint3.position.z = grasp_pose.position.z + 0.15
            waypoint3.orientation = grasp_pose.orientation
            
            # Compute Cartesian path through all waypoints
            lift_waypoints = [grasp_pose, waypoint1, waypoint2, waypoint3]  # Robot frame poses
            # Use MoveIt execution for better collision monitoring
            lift_trajectory = self._compute_cartesian_path(lift_waypoints, arm=arm, return_full_trajectory=True)
            if not self.execute_trajectory_moveit(lift_trajectory, arm=arm):
                raise RuntimeError(f'{arm}: Failed to execute lift trajectory through MoveIt')
            
            # CRITICAL: Verify object was actually grasped by checking if it moved with the gripper
            self.get_logger().info(f'{arm}: Verifying grasp - checking if {object_name} moved with gripper...')
            time.sleep(0.3)  # Wait for object pose to update in simulation
            post_lift_pose = self.get_pose(object_name)
            if not post_lift_pose:
                self.get_logger().error(f'{arm}: Cannot verify grasp - failed to get object pose after lift')
                return False
            
            # Get EE position after lift
            ee_data = self.get_ee_pose(arm)
            if not ee_data:
                self.get_logger().error(f'{arm}: Cannot verify grasp - failed to get EE pose after lift')
                return False
            ee_x, ee_y, ee_z = ee_data[0], ee_data[1], ee_data[2]
            
            # Check if object moved significantly (should be close to EE position now)
            obj_x, obj_y, obj_z = post_lift_pose['position']
            obj_size = post_lift_pose['size']
            distance_ee_to_obj = math.sqrt((ee_x - obj_x)**2 + (ee_y - obj_y)**2 + (ee_z - obj_z)**2)
            
            # ADAPTIVE tolerance based on object size - tall objects (upright solids) need more tolerance
            # Object center is further from gripper TCP for tall objects
            max_obj_dimension = max(obj_size)  # Get tallest dimension
            base_tolerance = 0.15  # 15cm base (gripper length + margin)
            if max_obj_dimension > 0.06:  # Tall objects (upright solids are 8cm tall)
                # For tall objects, center is further from gripper TCP
                max_expected_distance = base_tolerance + (max_obj_dimension / 2.0) + 0.05
                self.get_logger().info(f'{arm}: Using adaptive tolerance for tall object ({max_obj_dimension:.3f}m): {max_expected_distance:.3f}m')
            else:
                max_expected_distance = base_tolerance
            
            if distance_ee_to_obj > max_expected_distance:
                self.get_logger().error(f'{arm}: GRASP FAILED - Object {object_name} did not move with gripper!')
                self.get_logger().error(f'{arm}: EE position: ({ee_x:.3f}, {ee_y:.3f}, {ee_z:.3f}), Object position: ({obj_x:.3f}, {obj_y:.3f}, {obj_z:.3f})')
                self.get_logger().error(f'{arm}: Distance: {distance_ee_to_obj:.3f}m (expected < {max_expected_distance}m)')
                self.get_logger().error(f'{arm}: Object is still at original position - gripper did not grasp it!')
                
                # Detach object from planning scene (cleanup)
                scene = PlanningScene()
                scene.is_diff = True
                attached_obj = AttachedCollisionObject()
                attached_obj.object.id = object_name
                attached_obj.object.operation = CollisionObject.REMOVE
                eef_link = f'panda{1 if arm == "panda1" else 2}_hand'
                attached_obj.link_name = eef_link
                scene.robot_state.attached_collision_objects = [attached_obj]
                if self.joint_state:
                    scene.robot_state.joint_state = self.joint_state
                    scene.robot_state.is_diff = True
                self.planning_scene_pub.publish(scene)
                
                return False
            
            # Also check if object moved up (Z increased)
            original_z = obj_world_pos[2]
            current_z = obj_z
            z_delta = current_z - original_z
            # ADAPTIVE z_delta threshold - tall objects may have smaller relative lift due to physics
            min_z_delta = 0.03 if max_obj_dimension > 0.06 else 0.05  # 3cm for tall, 5cm for short
            if z_delta < min_z_delta:
                self.get_logger().error(f'{arm}: GRASP FAILED - Object {object_name} did not lift up!')
                self.get_logger().error(f'{arm}: Original Z: {original_z:.3f}m, Current Z: {current_z:.3f}m, Delta: {z_delta:.3f}m (expected > {min_z_delta:.3f}m)')
                
                # Detach object from planning scene (cleanup)
                scene = PlanningScene()
                scene.is_diff = True
                attached_obj = AttachedCollisionObject()
                attached_obj.object.id = object_name
                attached_obj.object.operation = CollisionObject.REMOVE
                eef_link = f'panda{1 if arm == "panda1" else 2}_hand'
                attached_obj.link_name = eef_link
                scene.robot_state.attached_collision_objects = [attached_obj]
                if self.joint_state:
                    scene.robot_state.joint_state = self.joint_state
                    scene.robot_state.is_diff = True
                self.planning_scene_pub.publish(scene)
                
                return False
            
            self.get_logger().info(f'{arm}: ✓ Grasp verified - Object {object_name} successfully grasped!')
            self.get_logger().info(f'{arm}: Object moved {z_delta:.3f}m up (min: {min_z_delta:.3f}m), distance to EE: {distance_ee_to_obj:.3f}m (max: {max_expected_distance:.3f}m)')
            
            self.get_logger().info(f'{arm}: Pick complete!')
            self.held_objects[arm] = object_name
            
            # Return final joint positions for potential J7 offset (used by prepare_for_insertion)
            # Get current joint state for potential use
            final_joints = None
            if self.joint_state:
                try:
                    final_joints = []
                    for joint_name in self.joint_names[arm]:
                        idx = self.joint_state.name.index(joint_name)
                        final_joints.append(self.joint_state.position[idx])
                except (ValueError, AttributeError):
                    pass
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'{arm}: Pick failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            
            # CRITICAL: Recovery movement - move arm to safe position so it's not stuck
            self.get_logger().info(f'{arm}: Attempting recovery movement to safe position...')
            try:
                import time
                # Open gripper first (release any partial grasp)
                self.send_gripper_goal(0.04, arm=arm)
                time.sleep(0.5)
                
                # Move to a safe position above the table
                world_safe = Pose()
                world_safe.position.x = 0.4
                world_safe.position.y = 0.15 if arm == 'panda1' else -0.15
                world_safe.position.z = 0.5  # 50cm above table
                qx, qy, qz, qw = _euler_to_quaternion(math.pi, 0.0, 0.0)
                world_safe.orientation.x = qx
                world_safe.orientation.y = qy
                world_safe.orientation.z = qz
                world_safe.orientation.w = qw
                
                safe_pose = self._transform_to_robot_frame(world_safe, arm)
                safe_pose.orientation = world_safe.orientation
                
                # Use IK for recovery (simpler than full planning)
                recovery_joints = self.compute_ik(safe_pose, arm=arm)
                if recovery_joints:
                    self.send_joint_trajectory(recovery_joints, arm=arm, seconds=2.0)
                    time.sleep(2.5)
                    self.get_logger().info(f'{arm}: Recovery movement complete - arm at safe position')
                else:
                    self.get_logger().warn(f'{arm}: Recovery IK failed, arm may still be stuck')
            except Exception as recovery_err:
                self.get_logger().warn(f'{arm}: Recovery movement failed: {recovery_err}')
            
            return False
    
    def place_object(self, arm: str, target_pos: List[float]) -> bool:
        """Place the held object at target (x, y, z_surface). Adapted for unified setup."""
        if not self.held_objects[arm]:
            self.get_logger().warn(f'{arm}: No object held! Assuming generic placement.')
            obj_height = 0.05
            obj_name = "unknown"
        else:
            obj_name = self.held_objects[arm]
            obj_data = self.get_pose(obj_name)
            if obj_data:
                obj_height = obj_data['size'][2]
            else:
                obj_height = 0.05

        try:
            import time
            
            tx, ty, tz_surface = target_pos
            
            # Target Position (WORLD FRAME)
            world_target = Pose()
            world_target.position.x = tx
            world_target.position.y = ty
            world_target.position.z = tz_surface + (obj_height / 2.0) + self.gripper_length  # TCP position
            
            # Orientation: Down with 45 deg yaw offset
            target_yaw_world = math.pi / 4.0
            qx, qy, qz, qw = _euler_to_quaternion(math.pi, 0.0, target_yaw_world)
            world_target.orientation.x = qx
            world_target.orientation.y = qy
            world_target.orientation.z = qz
            world_target.orientation.w = qw
            
            # 1. Move to Pre-Place (Hover) - WORLD FRAME
            world_pre_place = Pose()
            world_pre_place.position.x = tx
            world_pre_place.position.y = ty
            world_pre_place.position.z = world_target.position.z + 0.15
            world_pre_place.orientation = world_target.orientation
            
            self.get_logger().info(f'{arm}: Planning collision-free path to pre-place using OMPL (better obstacle avoidance for put_down)...')
            # Transform to robot base frame
            pre_place_pose = self._transform_to_robot_frame(world_pre_place, arm)
            pre_place_pose.orientation = world_pre_place.orientation
            # Use OMPL planning for put_down operations - it can navigate around obstacles better than Cartesian paths
            # Relax orientation constraints for put_down to make goal pose easier to reach
            try:
                planning_scene = self._setup_planning_scene(use_live_poses=True)
                robot_trajectory = self._plan_with_ompl(pre_place_pose, arm=arm, planner_id='RRTConnect', planning_scene=planning_scene, relax_orientation=True)
                if not self.execute_trajectory_moveit(robot_trajectory, arm=arm):
                    raise RuntimeError(f'{arm}: Failed to execute OMPL trajectory to pre-place')
            except Exception as e:
                self.get_logger().warn(f'{arm}: OMPL planning failed for pre-place ({e}), falling back to Cartesian path...')
                # Fallback to Cartesian path planning
                pre_place_joints = self.plan_to_pose(pre_place_pose, arm=arm, add_waypoints=True, refresh_scene=False)
            
            # 2. Descent to Place - Use OMPL planning for collision-free descent
            self.get_logger().info(f'{arm}: Descending to place using OMPL planning (collision-free)...')
            # Transform to robot base frame
            place_pose = self._transform_to_robot_frame(world_target, arm)
            place_pose.orientation = world_target.orientation
            
            # Use OMPL planning for descent - it can navigate around obstacles
            try:
                planning_scene = self._setup_planning_scene(use_live_poses=True)
                place_trajectory = self._plan_with_ompl(place_pose, arm=arm, planner_id='RRTConnect', planning_scene=planning_scene, relax_orientation=True)
                if not self.execute_trajectory_moveit(place_trajectory, arm=arm):
                    raise RuntimeError(f'{arm}: Failed to execute OMPL descent trajectory')
            except Exception as e:
                self.get_logger().warn(f'{arm}: OMPL planning for descent failed ({e}), falling back to Cartesian path...')
                # Fallback: Try simpler Cartesian path with just the target pose
                # Lower feasibility requirement for descent (80% instead of 99%)
                try:
                    place_trajectory = self._compute_cartesian_path([world_target], arm=arm, return_full_trajectory=True)
                    if not self.execute_trajectory_moveit(place_trajectory, arm=arm):
                        raise RuntimeError(f'{arm}: Failed to execute fallback descent trajectory')
                except RuntimeError as e2:
                    self.get_logger().error(f'{arm}: Both OMPL and Cartesian descent failed: {e2}')
                    raise
            
            # 3. Open Gripper IMMEDIATELY (object is already at place position)
            self.get_logger().info(f'{arm}: Releasing object...')
            self.send_gripper_goal(0.04, arm=arm)
            time.sleep(1.0)  # Wait for gripper to open
            
            # 4. Detach - UNIFIED PUBLISHER
            if obj_name != "unknown":
                scene = PlanningScene()
                scene.is_diff = True
                attached_obj = AttachedCollisionObject()
                attached_obj.object.id = obj_name
                attached_obj.object.operation = CollisionObject.REMOVE
                eef_link = f'panda{1 if arm == "panda1" else 2}_hand'
                attached_obj.link_name = eef_link
                scene.robot_state.attached_collision_objects = [attached_obj]
                # Include full unified joint state so MoveIt knows both arms' positions
                if self.joint_state:
                    scene.robot_state.joint_state = self.joint_state
                    scene.robot_state.is_diff = True
                self.planning_scene_pub.publish(scene)
                self.held_objects[arm] = None
                self.get_logger().info(f'{arm}: ✓ Object {obj_name} dropped successfully!')
            
            # 5. Retreat - WORLD FRAME (optional/non-critical - skip if fails)
            # Object is already dropped, so retreat failure shouldn't prevent success
            # This allows TAMP to proceed to next action (picking green_hollow)
            world_retreat = Pose()
            world_retreat.position.x = world_target.position.x
            world_retreat.position.y = world_target.position.y
            world_retreat.position.z = world_target.position.z + 0.15
            world_retreat.orientation = world_target.orientation
            
            self.get_logger().info(f'{arm}: Attempting to retreat (non-critical - will continue even if fails)...')
            try:
                # Try OMPL planning for better obstacle avoidance
                retreat_pose = self._transform_to_robot_frame(world_retreat, arm)
                retreat_pose.orientation = world_retreat.orientation
                
                planning_scene = self._setup_planning_scene(use_live_poses=True)
                ret_trajectory = self._plan_with_ompl(retreat_pose, arm=arm, planner_id='RRTConnect', planning_scene=planning_scene, relax_orientation=True)
                if not self.execute_trajectory_moveit(ret_trajectory, arm=arm):
                    raise RuntimeError(f'{arm}: Failed to execute OMPL retreat trajectory')
                self.get_logger().info(f'{arm}: ✓ Retreat successful')
            except Exception as e:
                self.get_logger().warn(f'{arm}: Retreat failed ({e}), but object is already dropped - proceeding to next TAMP action...')
                # Don't raise - object is already successfully placed, allow TAMP to continue
            
            return True

        except Exception as e:
            self.get_logger().error(f'{arm}: Place failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
    
    def _execute_task_plan(self, plan, arm: str) -> bool:
        """
        Execute a sequence of actions from TaskManager plan.
        
        Args:
            plan: List of Action objects from TaskManager.get_plan_to_grasp()
            arm: Arm to use for manipulation
        
        Returns:
            True if all actions succeeded, False otherwise
        """
        from pick_and_place.task_manager import Action, PredicateType
        
        # Prevent duplicate execution
        execution_key = f'{arm}_task_execution'
        if hasattr(self, '_task_execution_in_progress') and execution_key in self._task_execution_in_progress:
            self.get_logger().warn(f'{arm}: Task execution already in progress, ignoring duplicate call')
            return False
        
        if not hasattr(self, '_task_execution_in_progress'):
            self._task_execution_in_progress = set()
        self._task_execution_in_progress.add(execution_key)
        
        try:
            for action in plan:
                self.get_logger().info(f'{arm}: Executing TAMP action: {action}')
                
                if action.name == "unstack":
                    # unstack(arm, obj_top, obj_bottom)
                    obj_top = action.params[1]
                    obj_bottom = action.params[2]
                    
                    # Pick up the obstruction
                    if not self.pick_object(obj_top, arm=arm):
                        self.get_logger().error(f'{arm}: Failed to unstack {obj_top} from {obj_bottom}')
                        self._task_execution_in_progress.discard(execution_key)
                        return False
                    
                    # Update TaskManager state
                    self.task_manager.apply_action(action)
                    
                elif action.name == "put_down":
                    # put_down(arm, obj, location)
                    obj = action.params[1]
                    location = action.params[2]
                    
                    # Find a safe location on the table to place the object
                    if location == "table":
                        # Place on table - use a safe location within reach of the arm
                        # Panda 1 is at x=0, Panda 2 is at x=1.4
                        # Place closer to each arm's base to ensure reachability
                        if arm == "panda1":
                            safe_x = 0.4  # Close to Panda 1 (at x=0)
                            safe_y = 0.3  # On right side
                        else:  # panda2
                            safe_x = 1.0  # Close to Panda 2 (at x=1.4), within reach
                            safe_y = -0.3  # On left side
                        safe_z = self.table_height
                        
                        if not self.place_object(arm, [safe_x, safe_y, safe_z]):
                            self.get_logger().error(f'{arm}: Failed to put down {obj}')
                            self._task_execution_in_progress.discard(execution_key)
                            return False
                        
                        # Update object position in self.objects (object is now at new location)
                        with self.objects_lock:
                            if obj in self.objects:
                                self.objects[obj]['position'] = [safe_x, safe_y, safe_z]
                        
                        # Update TaskManager state
                        self.task_manager.apply_action(action)
                        
                        # Re-infer state from poses (object is now on table)
                        self.task_manager.update_state_from_poses()
                        
                        # CRITICAL: Move to safe height after put_down to avoid collisions when picking next object
                        # This ensures the arm is clear before starting next TAMP action
                        import time
                        self.get_logger().info(f'{arm}: Moving to safe height after put_down to avoid collisions...')
                        world_safe_height = Pose()
                        world_safe_height.position.x = safe_x
                        world_safe_height.position.y = safe_y
                        world_safe_height.position.z = 0.5  # 50cm above table - safe height
                        qx, qy, qz, qw = _euler_to_quaternion(math.pi, 0.0, 0.0)  # Gripper pointing down
                        world_safe_height.orientation.x = qx
                        world_safe_height.orientation.y = qy
                        world_safe_height.orientation.z = qz
                        world_safe_height.orientation.w = qw
                        
                        try:
                            safe_height_pose = self._transform_to_robot_frame(world_safe_height, arm)
                            safe_height_pose.orientation = world_safe_height.orientation
                            planning_scene = self._setup_planning_scene(use_live_poses=True)
                            safe_trajectory = self._plan_with_ompl(safe_height_pose, arm=arm, planner_id='RRTConnect', planning_scene=planning_scene, relax_orientation=True)
                            if self.execute_trajectory_moveit(safe_trajectory, arm=arm):
                                self.get_logger().info(f'{arm}: ✓ Moved to safe height after put_down')
                            else:
                                self.get_logger().warn(f'{arm}: Failed to move to safe height via MoveIt, trying direct...')
                                # Try direct movement as fallback
                                safe_joints = self.compute_ik(safe_height_pose, arm=arm)
                                if safe_joints:
                                    self.send_joint_trajectory(safe_joints, arm=arm, seconds=3.0)
                                    time.sleep(3.5)
                                    self.get_logger().info(f'{arm}: ✓ Moved to safe height via direct execution')
                        except Exception as e:
                            self.get_logger().warn(f'{arm}: Could not move to safe height ({e}), continuing anyway...')
                            # Don't fail the whole operation if safe height movement fails
                    else:
                        self.get_logger().warn(f'{arm}: put_down to non-table location not yet implemented')
                        self._task_execution_in_progress.discard(execution_key)
                        return False
                        
                elif action.name == "pick_up":
                    # pick_up(arm, obj) - This will be handled by the caller (prepare_for_insertion)
                    # Just update state
                    self.task_manager.apply_action(action)
                    
                else:
                    self.get_logger().error(f'{arm}: Unknown action type: {action.name}')
                    self._task_execution_in_progress.discard(execution_key)
                    return False
            
            return True
        finally:
            # Always remove execution lock, even if an exception occurs
            if hasattr(self, '_task_execution_in_progress'):
                self._task_execution_in_progress.discard(execution_key)
    
    def prepare_for_insertion(self, object_name: str, arm: str = 'panda2') -> bool:
        """Pick up object and move to assembly pose with 90deg rotation. Adapted for unified setup."""
        import time  # Import time for sleep calls
        
        # ============================================================
        # TAMP: Check if object is graspable and generate plan
        # ============================================================
        self.get_logger().info(f'{arm}: TAMP - Checking if {object_name} is graspable...')
        
        # CRITICAL: TaskManager state update - poses are already updated asynchronously by _sim_pose_callback
        # No need to refresh - just update the state from current object dictionary
        self.get_logger().info(f'{arm}: Updating TAMP world state from current object poses...')
        self.task_manager.update_state_from_poses()
        
        # Log world state (compact version for faster logging)
        state_str = self.task_manager.get_state_string()
        self.get_logger().info(f'{arm}: TAMP world state updated. Key predicates:')
        # Only log key predicates to reduce log spam
        for line in state_str.split('\n'):
            if any(keyword in line for keyword in ['on(', 'clear(', 'holding(', 'ON_TABLE']):
                self.get_logger().info(f'  {line}')
        self.get_logger().info(f'{arm}: Full state available (use get_state_string() for details)')
        
        # Get plan to make object graspable
        plan = self.task_manager.get_plan_to_grasp(target_obj=object_name, arm=arm)
        
        if len(plan) > 1 or (len(plan) == 1 and plan[0].name != "pick_up"):
            # Need to execute clearing actions first
            self.get_logger().info(f'{arm}: Object {object_name} is not clear. Executing clearing plan...')
            
            # Execute all actions except the final pick_up (we'll do that ourselves)
            actions_to_execute = [a for a in plan if a.name != "pick_up"]
            if not self._execute_task_plan(actions_to_execute, arm=arm):
                self.get_logger().error(f'{arm}: Failed to execute clearing plan for {object_name}')
                return False
            
            # Update state again after clearing
            self.task_manager.update_state_from_poses()
            self.get_logger().info(f'{arm}: World state after clearing:\n{self.task_manager.get_state_string()}')
        elif len(plan) == 0:
            self.get_logger().warn(f'{arm}: Empty plan returned for {object_name}')
            return False
        else:
            self.get_logger().info(f'{arm}: Object {object_name} is already clear, proceeding with pick...')
        
        # 1. Pick object
        if not self.pick_object(object_name, arm=arm):
            self.get_logger().error(f'{arm}: Failed to pick {object_name} - cannot proceed with Insert Prep')
            return False
        
        # Verify object is actually held (double-check)
        if self.held_objects.get(arm) != object_name:
            self.get_logger().error(f'{arm}: Object {object_name} is not marked as held after pick - cannot proceed')
            return False
        
        # Verify object moved with gripper (sanity check)
        time.sleep(0.2)  # Wait for pose updates
        obj_pose_after_pick = self.get_pose(object_name)
        ee_data = self.get_ee_pose(arm)
        if obj_pose_after_pick and ee_data:
            obj_x, obj_y, obj_z = obj_pose_after_pick['position']
            ee_x, ee_y, ee_z = ee_data[0], ee_data[1], ee_data[2]
            distance = math.sqrt((ee_x - obj_x)**2 + (ee_y - obj_y)**2 + (ee_z - obj_z)**2)
            if distance > 0.15:
                self.get_logger().error(f'{arm}: Object {object_name} is not close to EE after pick (distance: {distance:.3f}m) - grasp failed!')
                self.held_objects[arm] = None  # Clear held object status
                return False
            self.get_logger().info(f'{arm}: ✓ Verified {object_name} is properly held (distance to EE: {distance:.3f}m)')
        else:
            self.get_logger().warn(f'{arm}: Could not verify object position after pick, proceeding anyway...')
        
        # Update TaskManager state: object is now held
        from pick_and_place.task_manager import Predicate, PredicateType
        self.task_manager.add_predicate(Predicate(PredicateType.HOLDING, [arm, object_name]))
        self.task_manager.remove_predicate(Predicate(PredicateType.ON_TABLE, [object_name]))
        self.task_manager.remove_predicate(Predicate(PredicateType.CLEAR, [object_name]))
        self.task_manager.remove_predicate(Predicate(PredicateType.HAND_EMPTY, [arm]))
        
        # 2. Move to Assembly Pose - Transform to robot base frame
        # Moved back 5cm in X-axis (from 0.75 to 0.80) to give Panda 1 more space for picking
        world_assembly_pose = Pose()
        world_assembly_pose.position.x = 0.80  # Moved back from 0.75 to 0.80 (5cm further from Panda 1)
        world_assembly_pose.position.y = 0.0
        world_assembly_pose.position.z = 0.5
        
        # Orientation: Hole faces -X World (Towards Panda 1)
        qx, qy, qz, qw = _euler_to_quaternion(0.0, 1.5708, 0.0)
        world_assembly_pose.orientation.x = qx
        world_assembly_pose.orientation.y = qy
        world_assembly_pose.orientation.z = qz
        world_assembly_pose.orientation.w = qw
        
        # Transform to robot base frame
        assembly_pose = self._transform_to_robot_frame(world_assembly_pose, arm)
        assembly_pose.orientation = world_assembly_pose.orientation  # Keep original orientation
        
        # CRITICAL: Set up planning scene BEFORE motion planning to ensure table and objects are known
        self.get_logger().info(f'{arm}: Setting up planning scene with table and all objects before assembly motion...')
        self._setup_planning_scene()  # Add table + all 6 objects
        time.sleep(0.3)  # Wait for planning scene to be processed by MoveIt
        
        self.get_logger().info(f'{arm}: Planning collision-free path to Assembly Pose (Hole facing -X)...')
        try:
            # CRITICAL: Use collision-free path planning instead of direct IK execution
            # Direct IK only finds target joint angles - it doesn't plan a path or avoid obstacles!
            # Use Cartesian path planning (not OMPL) for simple, predictable straight-line movements
            assembly_joints = self.plan_to_pose(assembly_pose, arm=arm, add_waypoints=True, refresh_scene=False, use_ompl=False)
            # Note: plan_to_pose() now executes through MoveIt ExecuteTrajectory action internally
            
            # Step 3: Rotate J7 by 45 degrees (additional rotation after reaching assembly)
            self.get_logger().info(f'{arm}: Rotating J7 by additional 45 degrees...')
            # Get current joint positions after assembly motion
            if self.joint_state:
                current_joints = []
                for joint_name in self.joint_names[arm]:
                    try:
                        idx = self.joint_state.name.index(joint_name)
                        current_joints.append(self.joint_state.position[idx])
                    except ValueError:
                        current_joints.append(0.0)
                
                new_joints = list(current_joints)
                new_joints[6] += (math.pi / 4.0)
                self.send_joint_trajectory(new_joints, arm=arm, seconds=1.0)
            else:
                self.get_logger().warn(f'{arm}: No joint state available for J7 offset, skipping...')
            
            # CRITICAL: Ensure gripper stays closed after motion (prevent accidental release)
            self.get_logger().info(f'{arm}: Ensuring gripper maintains grip on object...')
            self.send_gripper_goal(0.01, arm=arm)  # Re-close gripper to maintain grip
            time.sleep(0.5)  # Brief wait to ensure gripper closes
            
            return True
        except Exception as e:
            self.get_logger().error(f'{arm}: Prep failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False

    def _compute_insertion_poses_frame_based(self, hollow_data: dict, solid_data: dict, gap: float, insertion_depth: float = 0.0) -> Pose:
        """
        Compute insertion pose using frame-based transforms.
        
        Args:
            hollow_data: Dict with 'position' [x,y,z], 'size' [lx,ly,lz], 'rpy' [r,p,y] for hollow object
            solid_data: Dict with 'position' [x,y,z], 'size' [lx,ly,lz] for solid object
            gap: Distance from hollow opening (pre-insert gap)
            insertion_depth: Depth of insertion (0.0 for pre-insert, >0 for actual insertion)
        
        Returns:
            gripper_pose_world: Gripper TCP pose in world frame
        
        Frame-based approach:
        - W: World frame
        - H: Hollow body frame (at geometric center)
        - I: Insertion frame (at hollow opening face, z-axis pointing outward)
        - S: Solid body frame (at geometric center)
        - Gs: Solid gripper TCP frame
        
        Transform chain:
        1. ^W T_H: Hollow pose in world (from live pose)
        2. ^W T_I = ^W T_H * ^H T_I: Insertion frame in world
        3. ^I T_S = Trans(0, 0, -solid_height/2 - gap - depth): Solid relative to insertion frame
        4. ^W T_S = ^W T_I * ^I T_S: Solid pose in world
        5. ^W T_Gs = ^W T_S * ^S T_Gs: Gripper TCP pose in world
        """
        # Extract hollow pose in world frame
        hx, hy, hz = hollow_data['position']
        hollow_size = hollow_data['size']  # [length_x=0.08, width_y=0.05, height_z=0.05]
        hollow_rpy = hollow_data.get('rpy', [0.0, 0.0, hollow_data.get('yaw', 0.0)])
        
        # Extract solid dimensions
        solid_size = solid_data['size']  # [width_x=0.025, depth_y=0.025, height_z=0.08]
        solid_height = solid_size[2]  # 0.08m
        
        # Step 1: Create ^W T_H (hollow pose in world)
        W_T_H = Pose()
        W_T_H.position.x = hx
        W_T_H.position.y = hy
        W_T_H.position.z = hz
        h_r, h_p, h_y = hollow_rpy
        qx, qy, qz, qw = _euler_to_quaternion(h_r, h_p, h_y)
        W_T_H.orientation.x = qx
        W_T_H.orientation.y = qy
        W_T_H.orientation.z = qz
        W_T_H.orientation.w = qw
        
        # Step 2: Create ^H T_I (hollow to insertion frame)
        # Insertion frame is at the opening face (center of opening)
        # Opening is on the face with normal pointing toward -X (toward Panda 1)
        # Position: half length forward along -X axis from hollow center
        H_T_I_pos = (-hollow_size[0]/2, 0.0, 0.0)  # Half length along -X (opening face)
        
        # Orientation: Insertion frame I's z-axis points outward (along -X of H, toward Panda 1)
        # Rotate I so its z-axis aligns with H's -X axis
        # +90 deg rotation around Y: I_z points along H_-X (toward Panda 1)
        # Rotation around Y: +90 deg makes z -> -x direction (correct for insertion from Panda 1)
        H_T_I_quat = _euler_to_quaternion(0.0, 1.5708, 0.0)  # +90 deg around Y: I_z points along H_-X (toward Panda 1)
        H_T_I = _pose_from_position_quaternion(H_T_I_pos, H_T_I_quat)
        
        # Step 3: Compute ^W T_I = ^W T_H * ^H T_I
        W_T_I = _compose_transforms(W_T_H, H_T_I)
        
        # Step 4: Create ^I T_S (solid relative to insertion frame)
        # I's z-axis points outward from hollow (toward where solid approaches from)
        # For pre-insert: gap distance away from opening along I's z-axis
        # For insert: (gap - insertion_depth) - solid is closer by insertion_depth
        # Position solid center: gap distance along -I_z (negative = toward hollow interior)
        offset_along_insertion = gap - insertion_depth  # Distance from insertion frame along insertion axis
        I_T_S_pos = (0.0, 0.0, -offset_along_insertion)  # Negative = along -I_z (toward hollow)
        I_T_S_quat = (0.0, 0.0, 0.0, 1.0)  # Solid orientation matches I
        I_T_S = _pose_from_position_quaternion(I_T_S_pos, I_T_S_quat)
        
        # Step 5: Compute ^W T_S = ^W T_I * ^I T_S
        W_T_S = _compose_transforms(W_T_I, I_T_S)
        
        # Step 6: Create ^S T_Gs (solid to gripper TCP)
        # Solid center = gripper TCP (no offset)
        S_T_Gs_pos = (0.0, 0.0, 0.0)
        S_T_Gs_quat = (0.0, 0.0, 0.0, 1.0)
        S_T_Gs = _pose_from_position_quaternion(S_T_Gs_pos, S_T_Gs_quat)
        
        # Step 7: Compute ^W T_Gs = ^W T_S * ^S T_Gs
        W_T_Gs = _compose_transforms(W_T_S, S_T_Gs)
        
        return W_T_Gs  # This is the gripper TCP pose in world frame
    
    def perform_insertion(self, object_name: str, arm: str = 'panda1') -> bool:
        """Pick object and insert into the hollow object held by Panda 2. Uses frame-based approach."""
        import time  # Import time at function level since pick_object also imports it
        
        # 1. Pick object
        if not self.pick_object(object_name, arm=arm):
            return False
        
        # 2. Retreat to Safe Position - Transform to robot base frame
        # CRITICAL: Make safe retreat position adaptive based on which side the solid object is on
        # Green objects are on positive Y (right side), Red objects are on negative Y (left side)
        # Retreat to the SAME side to avoid cross-workspace movement that causes collisions
        
        # Get the solid object's current position (after picking) to determine which side it's on
        time.sleep(0.1)  # Brief wait for pose updates after picking
        solid_pose_data = self.get_pose(object_name)
        if solid_pose_data:
            solid_current_y = solid_pose_data['position'][1]
            self.get_logger().info(f'{arm}: Solid object current position after pick: Y={solid_current_y:.3f}m')
        else:
            # Fallback: use initial object position
            solid_current_y = self.objects.get(object_name, {}).get('position', [0, 0, 0])[1]
            self.get_logger().info(f'{arm}: Using initial solid object position: Y={solid_current_y:.3f}m')
        
        # Determine safe retreat Y position based on object's side
        if solid_current_y >= 0:
            # Green objects (positive Y, right side) → retreat to positive Y (same side)
            safe_retreat_y = 0.15  # Right side
            side_str = "right (green side)"
        else:
            # Red objects (negative Y, left side) → retreat to negative Y (same side)
            safe_retreat_y = -0.15  # Left side
            side_str = "left (red side)"
        
        self.get_logger().info(f'{arm}: Retreating to safe position on {side_str} (Y={safe_retreat_y:.3f}m) to avoid cross-workspace movement...')
        
        world_safe_retreat_pose = Pose()
        world_safe_retreat_pose.position.x = 0.3
        world_safe_retreat_pose.position.y = safe_retreat_y  # Adaptive: same side as object (avoids crossing workspace)
        world_safe_retreat_pose.position.z = 0.6
        
        qx, qy, qz, qw = _euler_to_quaternion(math.pi, 0.0, 0.0)
        world_safe_retreat_pose.orientation.x = qx
        world_safe_retreat_pose.orientation.y = qy
        world_safe_retreat_pose.orientation.z = qz
        world_safe_retreat_pose.orientation.w = qw
        
        # Transform to robot base frame
        safe_retreat_pose = self._transform_to_robot_frame(world_safe_retreat_pose, arm)
        safe_retreat_pose.orientation = world_safe_retreat_pose.orientation  # Keep original orientation
            
        # 3. Get LIVE hollow object position from Gazebo (CRITICAL: Use live pose, not EE position)
        other_arm = 'panda2' if arm == 'panda1' else 'panda1'
        
        # Get the hollow object that Panda 2 is holding
        hollow_name = self.held_objects.get(other_arm)
        if not hollow_name:
            self.get_logger().error(f'{other_arm} is not holding any object. Cannot perform insertion.')
            return False
        
        # Get the solid object that Panda 1 is holding (for dimensions)
        solid_name = self.held_objects.get(arm)
        if not solid_name:
            self.get_logger().error(f'{arm} is not holding any object. Cannot perform insertion.')
            return False
        
        # Get live poses from Gazebo - CRITICAL: Use live hollow position
        time.sleep(0.2)  # Wait for pose updates
        hollow_data = self.get_pose(hollow_name)
        solid_data = self.get_pose(solid_name)
        
        if not hollow_data or not solid_data:
            self.get_logger().error(f'Could not get live poses for objects. Hollow: {hollow_data is not None}, Solid: {solid_data is not None}')
            return False
        
        # Extract hollow LIVE position (from Gazebo)
        hx, hy, hz = hollow_data['position']
        hollow_size = hollow_data['size']
        solid_size = solid_data['size']
        
        self.get_logger().info(f'{arm}: Hollow object LIVE position: X={hx:.3f}, Y={hy:.3f}, Z={hz:.3f}')
        
        # ============================================================
        # FRAME-BASED APPROACH: Compute poses using transform chain
        # ============================================================
        self.get_logger().info(f'{arm}: Using frame-based approach to compute insertion poses (gap={self.pre_insert_gap*100:.1f}cm)...')
        
        # Compute pre-insert pose (30cm gap) using frame-based transforms
        world_pre_insert_pose = self._compute_insertion_poses_frame_based(
            hollow_data, solid_data, gap=self.pre_insert_gap, insertion_depth=0.0
        )
        
        self.get_logger().info(f'{arm}: Frame-based pre-insert pose (world): X={world_pre_insert_pose.position.x:.3f}, Y={world_pre_insert_pose.position.y:.3f}, Z={world_pre_insert_pose.position.z:.3f}')
        self.get_logger().info(f'{arm}: Frame-based pre-insert orientation: qx={world_pre_insert_pose.orientation.x:.3f}, qy={world_pre_insert_pose.orientation.y:.3f}, qz={world_pre_insert_pose.orientation.z:.3f}, qw={world_pre_insert_pose.orientation.w:.3f}')
        
        # Transform to robot base frame
        pre_insert_pose = self._transform_to_robot_frame(world_pre_insert_pose, arm)
        pre_insert_pose.orientation = world_pre_insert_pose.orientation
        
        self.get_logger().info(f'{arm}: Frame-based pre-insert pose (robot frame): X={pre_insert_pose.position.x:.3f}, Y={pre_insert_pose.position.y:.3f}, Z={pre_insert_pose.position.z:.3f}')
        
        # Sanity check: If pre-insert X is too far (>0.85m), use simpler fallback
        if arm == 'panda1' and abs(pre_insert_pose.position.x) > 0.85:
            self.get_logger().warn(f'{arm}: Frame-based pre-insert pose X={pre_insert_pose.position.x:.3f}m is too far! Using simpler approach...')
            # Simple fallback: hollow_x - gap
            pre_insert_x = hx - self.pre_insert_gap
            world_pre_insert_pose = Pose()
            world_pre_insert_pose.position.x = pre_insert_x
            world_pre_insert_pose.position.y = hy
            world_pre_insert_pose.position.z = hz
            qx, qy, qz, qw = _euler_to_quaternion(0.0, 1.5708, 0.0)
            world_pre_insert_pose.orientation.x = qx
            world_pre_insert_pose.orientation.y = qy
            world_pre_insert_pose.orientation.z = qz
            world_pre_insert_pose.orientation.w = qw
            pre_insert_pose = self._transform_to_robot_frame(world_pre_insert_pose, arm)
            pre_insert_pose.orientation = world_pre_insert_pose.orientation
            self.get_logger().info(f'{arm}: Using simpler pre-insert pose: X={pre_insert_pose.position.x:.3f}m')
        
        # Compute insert pose (20cm insertion depth) using frame-based transforms
        world_insert_pose = self._compute_insertion_poses_frame_based(
            hollow_data, solid_data, gap=self.pre_insert_gap, insertion_depth=self.insert_depth
        )
        
        self.get_logger().info(f'{arm}: Frame-based insert pose (world): X={world_insert_pose.position.x:.3f}, Y={world_insert_pose.position.y:.3f}, Z={world_insert_pose.position.z:.3f}')
        
        # Transform to robot base frame
        insert_pose = self._transform_to_robot_frame(world_insert_pose, arm)
        insert_pose.orientation = world_insert_pose.orientation
        
        self.get_logger().info(f'{arm}: Frame-based insert pose (robot frame): X={insert_pose.position.x:.3f}, Y={insert_pose.position.y:.3f}, Z={insert_pose.position.z:.3f}')
        
        # Sanity check: If insert pose X is too far, compute from pre-insert + insertion depth
        if arm == 'panda1' and abs(insert_pose.position.x) > 0.85:
            self.get_logger().warn(f'{arm}: Frame-based insert pose X={insert_pose.position.x:.3f}m is too far! Computing from pre-insert position...')
            # Compute insert pose as pre-insert + insertion depth forward
            world_pre_insert_x = hx - self.pre_insert_gap
            world_insert_pose.position.x = world_pre_insert_x + self.insert_depth
            world_insert_pose.position.y = hy
            world_insert_pose.position.z = hz
            insert_pose = self._transform_to_robot_frame(world_insert_pose, arm)
            insert_pose.orientation = world_insert_pose.orientation
            self.get_logger().info(f'{arm}: Using simpler insert pose: X={insert_pose.position.x:.3f}m')
        
        try:
            # CRITICAL: Ensure ALL collision objects are in planning scene:
            # 1. All 6 objects (red_block, green_block, red_solid, green_solid, red_hollow, green_hollow)
            # 2. Table
            # 3. Hollow object held by Panda 2
            # 4. Both arms (automatic with unified MoveIt, but we ensure joint state is current)
            self.get_logger().info(f'{arm}: Setting up complete planning scene with all objects and table...')
            self._setup_planning_scene()  # Add all 6 objects + table
            time.sleep(0.2)  # Wait for scene to be processed
            
            # Add hollow object held by Panda 2 as a collision object
            self.get_logger().info(f'[COLLISION DEBUG] ====== COLLISION SETUP FOR INSERTION ======')
            self.get_logger().info(f'[COLLISION DEBUG] Inserting arm: {arm}, Other arm: {other_arm}')
            self.get_logger().info(f'[COLLISION DEBUG] Hollow object: {hollow_name}, Solid object: {solid_name}')
            self.get_logger().info(f'[COLLISION DEBUG] Hollow position (live): X={hx:.3f}, Y={hy:.3f}, Z={hz:.3f}')
            self._add_held_object_as_collision(other_arm, hollow_name, hollow_data)
            time.sleep(0.2)  # Wait for planning scene to be processed
            self.get_logger().info(f'[COLLISION DEBUG] Collision object published, waiting for MoveIt to process...')
            
            # Step 2a: Move to safe position (already transformed above) - OLD LOGIC: Direct IK
            self.get_logger().info(f'{arm}: Moving to safe retreat position...')
            joints_safe = self.compute_ik(safe_retreat_pose, arm=arm)  # Uses robot base frame by default
            self.send_joint_trajectory(joints_safe, arm=arm, seconds=3.0)
            
            # Step 3: Plan collision-free path from safe position to align in front of hole
            # CRITICAL: Refresh planning scene before planning to ensure all objects are current
            # Re-read LIVE object poses to ensure we have the latest positions
            time.sleep(0.2)  # Wait for pose updates
            updated_hollow_data = self.get_pose(hollow_name)
            if updated_hollow_data:
                hollow_data = updated_hollow_data  # Use updated pose
                self.get_logger().info(f'{arm}: Refreshed hollow pose: ({hollow_data["position"][0]:.3f}, {hollow_data["position"][1]:.3f}, {hollow_data["position"][2]:.3f})')
            
            self._setup_planning_scene(use_live_poses=True)  # Refresh all objects with latest poses
            
            # DEBUG: Verify joint state includes both arms before collision object update
            if self.joint_state:
                panda1_joints = [name for name in self.joint_state.name if name.startswith('panda1_')]
                panda2_joints = [name for name in self.joint_state.name if name.startswith('panda2_')]
                self.get_logger().info(f'[COLLISION DEBUG] Unified joint state: {len(panda1_joints)} panda1 joints, {len(panda2_joints)} panda2 joints')
                if len(panda2_joints) == 0:
                    self.get_logger().error(f'[COLLISION DEBUG] WARNING: No panda2 joints in unified joint state! MoveIt may not know Panda 2 position!')
            else:
                self.get_logger().error(f'[COLLISION DEBUG] WARNING: No joint state available! MoveIt may not know arm positions!')
            
            # Get fresh EE pose for other arm before updating collision object
            fresh_ee_data = self.get_ee_pose(other_arm)
            if fresh_ee_data:
                fresh_ee_x, fresh_ee_y, fresh_ee_z = fresh_ee_data[0], fresh_ee_data[1], fresh_ee_data[2]
                self.get_logger().info(f'[COLLISION DEBUG] Fresh {other_arm} EE pose before collision update: X={fresh_ee_x:.3f}, Y={fresh_ee_y:.3f}, Z={fresh_ee_z:.3f}')
            else:
                self.get_logger().error(f'[COLLISION DEBUG] WARNING: Cannot get fresh EE pose for {other_arm}! Using stale pose!')
            
            self._add_held_object_as_collision(other_arm, hollow_name, hollow_data)  # Update held object
            time.sleep(0.5)  # Wait for planning scene to be processed by MoveIt
            
            # Step 3: Plan collision-free path from safe position to align in front of hole
            # CRITICAL: Refresh planning scene before planning to ensure all objects are current
            # Re-read LIVE object poses to ensure we have the latest positions
            time.sleep(0.2)  # Wait for pose updates
            updated_hollow_data = self.get_pose(hollow_name)
            if updated_hollow_data:
                hollow_data = updated_hollow_data  # Use updated pose
                self.get_logger().info(f'{arm}: Refreshed hollow pose: ({hollow_data["position"][0]:.3f}, {hollow_data["position"][1]:.3f}, {hollow_data["position"][2]:.3f})')
            
            self._setup_planning_scene()  # Refresh all objects with latest poses
            self._add_held_object_as_collision(other_arm, hollow_name, hollow_data)  # Update held object
            time.sleep(0.5)  # Wait for planning scene to be processed by MoveIt
            
            self.get_logger().info(f'{arm}: Planning collision-free path around Panda 2 and hollow object...')
            self.get_logger().info(f'{arm}: Using unified joint state (both arms) for collision checking...')
            
            # Step 3: Compute collision-free path to pre-insert position (30cm in front of hollow)
            self.get_logger().info(f'{arm}: Planning collision-free path to pre-insert position (30cm in front of hollow)...')
            
            # Check if computed pose is too far (likely unreachable)
            # Panda 1 reach is ~0.85m, so if x > 0.85m in robot frame, fallback to simpler approach
            if arm == 'panda1' and abs(pre_insert_pose.position.x) > 0.85:
                self.get_logger().warn(f'{arm}: Frame-based pose appears unreachable (x={pre_insert_pose.position.x:.3f}m). Using simpler fallback approach...')
                # Fallback: Simple approach - hollow_x - gap
                pre_insert_x = hx - self.pre_insert_gap
                world_pre_insert_pose = Pose()
                world_pre_insert_pose.position.x = pre_insert_x
                world_pre_insert_pose.position.y = hy
                world_pre_insert_pose.position.z = hz
                qx, qy, qz, qw = _euler_to_quaternion(0.0, 1.5708, 0.0)  # Pitch=90deg
                world_pre_insert_pose.orientation.x = qx
                world_pre_insert_pose.orientation.y = qy
                world_pre_insert_pose.orientation.z = qz
                world_pre_insert_pose.orientation.w = qw
                pre_insert_pose = self._transform_to_robot_frame(world_pre_insert_pose, arm)
                pre_insert_pose.orientation = world_pre_insert_pose.orientation
                self.get_logger().info(f'{arm}: Fallback pre-insert pose (robot frame): X={pre_insert_pose.position.x:.3f}, Y={pre_insert_pose.position.y:.3f}, Z={pre_insert_pose.position.z:.3f}')
            
            # Try IK first (simpler, might work if path is clear) - OLD LOGIC
            try:
                self.get_logger().info(f'{arm}: Attempting IK to pre-insert position...')
                pre_insert_joints = self.compute_ik(pre_insert_pose, arm=arm)
                self.send_joint_trajectory(pre_insert_joints, arm=arm, seconds=4.0)
                self.get_logger().info(f'{arm}: Successfully reached pre-insert position via IK!')
            except Exception as ik_error:
                # If IK fails, use Cartesian path planning (better for collision avoidance)
                self.get_logger().warn(f'{arm}: IK failed, trying Cartesian path planning: {ik_error}')
                
                # Use Cartesian path from current position (after safe retreat) to pre-insert
                waypoints_to_pre_insert = [safe_retreat_pose, pre_insert_pose]
                joints_to_pre_insert = self._compute_cartesian_path(waypoints_to_pre_insert, arm=arm)
                
                if joints_to_pre_insert:
                    self.send_joint_trajectory(joints_to_pre_insert, arm=arm, seconds=5.0)
                    self.get_logger().info(f'{arm}: Successfully reached pre-insert position via Cartesian path!')
                else:
                    # Final fallback: Try simpler approach
                    self.get_logger().warn(f'{arm}: Cartesian path also failed. Trying simpler fallback approach...')
                    pre_insert_x = hx - self.pre_insert_gap
                    world_pre_insert_pose = Pose()
                    world_pre_insert_pose.position.x = pre_insert_x
                    world_pre_insert_pose.position.y = hy
                    world_pre_insert_pose.position.z = hz
                    qx, qy, qz, qw = _euler_to_quaternion(0.0, 1.5708, 0.0)
                    world_pre_insert_pose.orientation.x = qx
                    world_pre_insert_pose.orientation.y = qy
                    world_pre_insert_pose.orientation.z = qz
                    world_pre_insert_pose.orientation.w = qw
                    pre_insert_pose = self._transform_to_robot_frame(world_pre_insert_pose, arm)
                    pre_insert_pose.orientation = world_pre_insert_pose.orientation
                    
                    # Try IK with fallback pose
                    try:
                        pre_insert_joints = self.compute_ik(pre_insert_pose, arm=arm)
                        self.send_joint_trajectory(pre_insert_joints, arm=arm, seconds=4.0)
                        self.get_logger().info(f'{arm}: Successfully reached pre-insert position via fallback IK!')
                    except Exception as fallback_error:
                        raise RuntimeError(f'Both frame-based and fallback approaches failed. Last error: {fallback_error}')
            
            # Frame-based approach: Poses are already correctly aligned via transform chain
            # No manual alignment needed - trust the frame-based transforms
            self.get_logger().info(f'{arm}: Frame-based approach: Poses are automatically aligned via transform chain.')
            time.sleep(0.3)  # Brief wait to ensure previous motion is complete
            
            # CRITICAL: Apply 45-degree J7 offset IMMEDIATELY after reaching pre-insert position
            # This must happen BEFORE any insertion calculations or movements
            self.get_logger().info(f'{arm}: Applying 45-degree J7 offset immediately after reaching pre-insert position...')
            j7_offset_applied = False
            if self.joint_state:
                try:
                    # Get current joint positions for this arm
                    current_joints = []
                    for joint_name in self.joint_names[arm]:
                        idx = self.joint_state.name.index(joint_name)
                        current_joints.append(self.joint_state.position[idx])
                    
                    # Apply 45-degree offset to J7 (index 6)
                    new_joints = list(current_joints)
                    old_j7 = current_joints[6]
                    new_joints[6] += (math.pi / 4.0)  # +45 degrees
                    
                    self.get_logger().info(f'{arm}: Applying J7 offset: {old_j7:.3f} rad -> {new_joints[6]:.3f} rad (+45 deg)')
                    self.send_joint_trajectory(new_joints, arm=arm, seconds=1.5)
                    time.sleep(1.0)  # Wait for rotation to complete
                    
                    # Verify the offset was applied
                    if self.joint_state:
                        try:
                            j7_idx = self.joint_state.name.index(self.joint_names[arm][6])
                            actual_j7 = self.joint_state.position[j7_idx]
                            j7_diff = abs(actual_j7 - old_j7)
                            self.get_logger().info(f'{arm}: J7 offset verified. Current J7: {actual_j7:.3f} rad (difference: {j7_diff:.3f} rad)')
                            if j7_diff > 0.5:  # At least 0.5 rad (~28 deg) difference confirms rotation
                                j7_offset_applied = True
                            else:
                                self.get_logger().warn(f'{arm}: J7 offset may not have been applied! Current J7: {actual_j7:.3f}, Start: {old_j7:.3f}')
                        except (ValueError, IndexError) as e2:
                            self.get_logger().warn(f'{arm}: Could not verify J7 offset: {e2}')
                    j7_offset_applied = True
                except (ValueError, IndexError) as e:
                    self.get_logger().error(f'{arm}: Failed to apply J7 offset: {e}')
                    import traceback
                    self.get_logger().error(traceback.format_exc())
            
            if not j7_offset_applied:
                self.get_logger().warn(f'{arm}: WARNING - J7 offset may not have been applied! Proceeding anyway...')
            else:
                self.get_logger().info(f'{arm}: J7 offset successfully applied. Ready for insertion calculations.')
            
            time.sleep(0.3)  # Brief wait after J7 offset
            
            # Step 4: Move forward using frame-based insert pose - OLD INCREMENTAL LOGIC
            # CRITICAL: Refresh planning scene right before insertion to ensure latest obstacle positions
            self.get_logger().info(f'{arm}: Refreshing planning scene before insertion...')
            time.sleep(0.1)
            updated_hollow_data = self.get_pose(hollow_name)
            if updated_hollow_data:
                hollow_data = updated_hollow_data
                
                # CRITICAL: Update insert pose to account for actual yaw difference between solid and hollow
                # Get current solid yaw (from pose or initial config)
                solid_yaw = solid_data.get('rpy', [0.0, 0.0, solid_data.get('yaw', 0.0)])[2]
                hollow_yaw = hollow_data.get('rpy', [0.0, 0.0, hollow_data.get('yaw', 0.0)])[2]
                yaw_diff = hollow_yaw - solid_yaw
                
                self.get_logger().info(f'{arm}: Yaw alignment check - Solid yaw: {solid_yaw:.4f} rad ({solid_yaw*180/math.pi:.2f} deg), Hollow yaw: {hollow_yaw:.4f} rad ({hollow_yaw*180/math.pi:.2f} deg), Difference: {yaw_diff:.4f} rad ({yaw_diff*180/math.pi:.2f} deg)')
                
                # Recompute insert pose with updated hollow pose (which includes actual yaw)
                # The frame-based approach already uses hollow yaw through transform chain, so this ensures latest yaw is used
                world_insert_pose = self._compute_insertion_poses_frame_based(
                    hollow_data, solid_data, gap=self.pre_insert_gap, insertion_depth=self.insert_depth
                )
                insert_pose = self._transform_to_robot_frame(world_insert_pose, arm)
                insert_pose.orientation = world_insert_pose.orientation
                
                self.get_logger().info(f'{arm}: Updated insert pose with latest hollow yaw: X={insert_pose.position.x:.3f}, Y={insert_pose.position.y:.3f}, Z={insert_pose.position.z:.3f}')
            
            self._setup_planning_scene()  # Refresh all objects
            self._add_held_object_as_collision(other_arm, hollow_name, hollow_data)  # Update held object
            time.sleep(0.3)  # Wait for planning scene to be processed
            
            # Get current pose for Cartesian path starting point (AFTER J7 offset)
            current_ee_data = self.get_ee_pose(arm)
            if not current_ee_data:
                self.get_logger().warn(f'{arm}: Cannot get current EE pose, using pre_insert_pose as fallback')
                current_ee_pose_robot = pre_insert_pose
            else:
                current_ee_pose_world = Pose()
                current_ee_pose_world.position.x = current_ee_data[0]
                current_ee_pose_world.position.y = current_ee_data[1]
                current_ee_pose_world.position.z = current_ee_data[2]
                qx, qy, qz, qw = _euler_to_quaternion(current_ee_data[3], current_ee_data[4], current_ee_data[5])
                current_ee_pose_world.orientation.x = qx
                current_ee_pose_world.orientation.y = qy
                current_ee_pose_world.orientation.z = qz
                current_ee_pose_world.orientation.w = qw
                current_ee_pose_robot = self._transform_to_robot_frame(current_ee_pose_world, arm)
                # CRITICAL: Use the actual orientation AFTER J7 rotation (includes the 45deg offset)
                current_ee_pose_robot.orientation = current_ee_pose_world.orientation
            
            # Use frame-based insert pose (already computed with correct yaw alignment and 20cm insertion depth)
            self.get_logger().info(f'{arm}: Using frame-based insert pose: X={insert_pose.position.x:.3f}, Y={insert_pose.position.y:.3f}, Z={insert_pose.position.z:.3f}')
            self.get_logger().info(f'{arm}: Current position after J7 offset: X={current_ee_pose_robot.position.x:.3f}, Y={current_ee_pose_robot.position.y:.3f}, Z={current_ee_pose_robot.position.z:.3f}')
            
            # CRITICAL: Break insertion into smaller increments to avoid obstacles - OLD INCREMENTAL LOGIC
            # Cartesian paths can only go straight, so we'll move forward in smaller steps
            # Calculate initial distance to target
            dx = insert_pose.position.x - current_ee_pose_robot.position.x
            dy = insert_pose.position.y - current_ee_pose_robot.position.y
            dz = insert_pose.position.z - current_ee_pose_robot.position.z
            initial_distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            self.get_logger().info(f'{arm}: Total insertion distance: {initial_distance*100:.1f}cm. Breaking into 2cm increments...')
            
            step_size = 0.02  # 2cm increments
            current_pose = current_ee_pose_robot
            
            # Move forward in small steps until we reach the target
            max_iterations = 20  # Safety limit
            iteration = 0
            while iteration < max_iterations:
                iteration += 1
                # Recalculate distance and direction from CURRENT position to target
                dx = insert_pose.position.x - current_pose.position.x
                dy = insert_pose.position.y - current_pose.position.y
                dz = insert_pose.position.z - current_pose.position.z
                remaining_distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if remaining_distance < 0.005:  # Within 5mm of target
                    self.get_logger().info(f'{arm}: Reached target! Remaining distance: {remaining_distance*100:.2f}cm')
                    break
                
                # Normalize direction vector
                if remaining_distance < 0.001:
                    break  # Already at target
                
                direction_x = dx / remaining_distance
                direction_y = dy / remaining_distance
                direction_z = dz / remaining_distance
                
                # Step distance (2cm or remaining distance, whichever is smaller)
                step_distance = min(step_size, remaining_distance)
                
                # Create next waypoint (current + step in direction of target)
                next_pose = Pose()
                next_pose.position.x = current_pose.position.x + direction_x * step_distance
                next_pose.position.y = current_pose.position.y + direction_y * step_distance
                next_pose.position.z = current_pose.position.z + direction_z * step_distance
                # CRITICAL: Use current orientation (which includes J7 offset) instead of insert_pose orientation
                # This preserves the 45-degree J7 rotation throughout the linear movement
                next_pose.orientation = current_pose.orientation  # Keep current orientation (includes J7 offset)
                
                # Try to move to this waypoint
                try:
                    waypoints_step = [current_pose, next_pose]
                    joints_step = self._compute_cartesian_path(waypoints_step, arm=arm)
                    if joints_step:
                        self.send_joint_trajectory(joints_step, arm=arm, seconds=2.0)
                        remaining_distance -= step_distance
                        self.get_logger().info(f'{arm}: Moved forward {step_distance*100:.1f}cm. Remaining: {remaining_distance*100:.1f}cm')
                        
                        # Update current pose for next step
                        time.sleep(0.2)
                        updated_ee_data = self.get_ee_pose(arm)
                        if updated_ee_data:
                            current_pose_world = Pose()
                            current_pose_world.position.x = updated_ee_data[0]
                            current_pose_world.position.y = updated_ee_data[1]
                            current_pose_world.position.z = updated_ee_data[2]
                            qx, qy, qz, qw = _euler_to_quaternion(updated_ee_data[3], updated_ee_data[4], updated_ee_data[5])
                            current_pose_world.orientation.x = qx
                            current_pose_world.orientation.y = qy
                            current_pose_world.orientation.z = qz
                            current_pose_world.orientation.w = qw
                            current_pose = self._transform_to_robot_frame(current_pose_world, arm)
                            # CRITICAL: Use actual current orientation (which includes J7 offset) instead of overwriting
                            current_pose.orientation = current_pose_world.orientation  # Keep current orientation (includes J7 offset)
                            
                            # Recalculate remaining distance from current position
                            dx = insert_pose.position.x - current_pose.position.x
                            dy = insert_pose.position.y - current_pose.position.y
                            dz = insert_pose.position.z - current_pose.position.z
                            remaining_distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                        else:
                            # Assume we moved successfully, update remaining distance
                            remaining_distance -= step_distance
                    else:
                        self.get_logger().warn(f'{arm}: Cartesian path failed for {step_distance*100:.1f}cm step. Trying IK...')
                        # Try IK as fallback
                        try:
                            joints_ik = self.compute_ik(next_pose, arm=arm)
                            self.send_joint_trajectory(joints_ik, arm=arm, seconds=2.0)
                            remaining_distance -= step_distance
                            time.sleep(0.2)
                            # Update current pose
                            updated_ee_data = self.get_ee_pose(arm)
                            if updated_ee_data:
                                current_pose_world = Pose()
                                current_pose_world.position.x = updated_ee_data[0]
                                current_pose_world.position.y = updated_ee_data[1]
                                current_pose_world.position.z = updated_ee_data[2]
                                qx, qy, qz, qw = _euler_to_quaternion(updated_ee_data[3], updated_ee_data[4], updated_ee_data[5])
                                current_pose_world.orientation.x = qx
                                current_pose_world.orientation.y = qy
                                current_pose_world.orientation.z = qz
                                current_pose_world.orientation.w = qw
                                current_pose = self._transform_to_robot_frame(current_pose_world, arm)
                                # CRITICAL: Use actual current orientation (which includes J7 offset)
                                current_pose.orientation = current_pose_world.orientation  # Keep current orientation (includes J7 offset)
                                # Recalculate remaining distance
                                dx = insert_pose.position.x - current_pose.position.x
                                dy = insert_pose.position.y - current_pose.position.y
                                dz = insert_pose.position.z - current_pose.position.z
                                remaining_distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                        except Exception as ik_err:
                            self.get_logger().error(f'{arm}: IK also failed: {ik_err}. Stopping incremental insertion.')
                            break
                except Exception as step_err:
                    self.get_logger().warn(f'{arm}: Step failed: {step_err}. Trying direct IK to final target...')
                    # Try direct IK to final target as last resort
                    try:
                        joints_final = self.compute_ik(insert_pose, arm=arm)
                        self.send_joint_trajectory(joints_final, arm=arm, seconds=3.0)
                        self.get_logger().info(f'{arm}: Successfully reached insert pose via direct IK!')
                        remaining_distance = 0  # Mark as complete
                        break
                    except Exception as final_err:
                        self.get_logger().error(f'{arm}: Final IK also failed: {final_err}')
                        return False
            
            # Check final distance and attempt final correction if needed
            time.sleep(0.3)
            final_ee_data = self.get_ee_pose(arm)
            if final_ee_data:
                final_pose_world = Pose()
                final_pose_world.position.x = final_ee_data[0]
                final_pose_world.position.y = final_ee_data[1]
                final_pose_world.position.z = final_ee_data[2]
                final_pose_robot = self._transform_to_robot_frame(final_pose_world, arm)
                final_dx = insert_pose.position.x - final_pose_robot.position.x
                final_dy = insert_pose.position.y - final_pose_robot.position.y
                final_dz = insert_pose.position.z - final_pose_robot.position.z
                final_distance = math.sqrt(final_dx*final_dx + final_dy*final_dy + final_dz*final_dz)
                
                # If close but not quite there, try one more IK attempt to complete insertion
                if 0.005 < final_distance <= 0.03:  # Between 0.5-3cm away
                    self.get_logger().info(f'{arm}: Close to target (distance: {final_distance*100:.2f}cm). Attempting final IK to complete insertion...')
                    try:
                        final_joints = self.compute_ik(insert_pose, arm=arm)
                        self.send_joint_trajectory(final_joints, arm=arm, seconds=2.0)
                        time.sleep(0.3)
                        self.get_logger().info(f'{arm}: Final correction completed!')
                    except Exception as final_ik_err:
                        self.get_logger().warn(f'{arm}: Final IK correction failed: {final_ik_err}')
                
                if final_distance <= 0.02:  # Within 2cm is acceptable
                    self.get_logger().info(f'{arm}: Successfully completed insertion! Final distance: {final_distance*100:.2f}cm')
                else:
                    self.get_logger().warn(f'{arm}: Insertion incomplete. Final distance: {final_distance*100:.1f}cm')
            else:
                self.get_logger().warn(f'{arm}: Could not verify final insertion distance (EE pose unavailable)')
            
            self.get_logger().info(f'{arm}: Insert sequence complete! Used frame-based approach for perfect alignment and straight insertion.')
            
            self.get_logger().info(f'{arm}: ✓ Insert sequence complete!')
            return True
            
            # OLD CODE MARKER - everything below is dead code to be removed
            alignment_offset_z_DEAD = 0.009  # +1cm in Z direction (toward +Z)
            
            # Update insert pose with alignment offsets
            insert_pose.position.y += 0  # alignment_offset_y
            insert_pose.position.z += alignment_offset_z_DEAD
            self.get_logger().info(f'{arm}: Adjusted insert pose: Y={insert_pose.position.y:.3f} ({alignment_offset_y*100:.1f}cm), Z={insert_pose.position.z:.3f} (+{alignment_offset_z*100:.1f}cm)')
            
            # CRITICAL: Break insertion into smaller increments to avoid obstacles
            # Cartesian paths can only go straight, so we'll move forward in smaller steps
            # Calculate initial distance to target (after alignment offsets)
            dx = insert_pose.position.x - current_ee_pose_robot.position.x
            dy = insert_pose.position.y - current_ee_pose_robot.position.y
            dz = insert_pose.position.z - current_ee_pose_robot.position.z
            initial_distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            self.get_logger().info(f'{arm}: Total insertion distance: {initial_distance*100:.1f}cm. Breaking into 2cm increments...')
            
            step_size = 0.02  # 2cm increments
            current_pose = current_ee_pose_robot
            
            # Move forward in small steps until we reach the target
            max_iterations = 20  # Safety limit
            iteration = 0
            while iteration < max_iterations:
                iteration += 1  # Increment at start of loop iteration
                # Recalculate distance and direction from CURRENT position to target
                dx = insert_pose.position.x - current_pose.position.x
                dy = insert_pose.position.y - current_pose.position.y
                dz = insert_pose.position.z - current_pose.position.z
                remaining_distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if remaining_distance < 0.005:  # Within 5mm of target
                    self.get_logger().info(f'{arm}: Reached target! Remaining distance: {remaining_distance*100:.2f}cm')
                    break
                
                # Normalize direction vector
                if remaining_distance < 0.001:
                    break  # Already at target
                
                direction_x = dx / remaining_distance
                direction_y = dy / remaining_distance
                direction_z = dz / remaining_distance
                
                # Step distance (2cm or remaining distance, whichever is smaller)
                step_distance = min(step_size, remaining_distance)
                
                # Create next waypoint (current + step in direction of target)
                next_pose = Pose()
                next_pose.position.x = current_pose.position.x + direction_x * step_distance
                next_pose.position.y = current_pose.position.y + direction_y * step_distance
                next_pose.position.z = current_pose.position.z + direction_z * step_distance
                # CRITICAL: Use current orientation (which includes J7 offset) instead of insert_pose orientation
                # This preserves the 45-degree J7 rotation throughout the linear movement
                next_pose.orientation = current_pose.orientation  # Keep current orientation (includes J7 offset)
                
                # Note: For Cartesian paths (straight-line), we don't need to refresh planning scene every step
                # Only refresh every 5 steps (10cm) to reduce latency while still catching major changes
                if iteration % 5 == 0:
                    self.get_logger().info(f'{arm}: Refreshing planning scene (step {iteration+1})...')
                    self._setup_planning_scene(use_live_poses=True)
                    time.sleep(0.1)  # Brief wait
                
                # CRITICAL: Use Cartesian path planning (straight-line) for insertion - NOT OMPL!
                # OMPL finds curved paths through joint space which causes arm reconfigurations and rotation
                # For insertion, we need straight-line movement with fixed orientation
                try:
                    # Use Cartesian path planning for straight-line movement to next waypoint
                    self.get_logger().info(f'{arm}: Using Cartesian path for step {iteration+1} ({step_distance*100:.1f}cm forward)...')
                    # Create 2-pose Cartesian path: current -> next
                    waypoints_cartesian = [current_pose, next_pose]
                    trajectory = self._compute_cartesian_path(waypoints_cartesian, arm=arm, return_full_trajectory=True)
                    if trajectory:
                        self.execute_trajectory_moveit(trajectory, arm=arm)
                        remaining_distance -= step_distance
                        self.get_logger().info(f'{arm}: Moved forward {step_distance*100:.1f}cm using Cartesian path. Remaining: {remaining_distance*100:.1f}cm')
                    else:
                        raise RuntimeError(f'Cartesian path planning failed for step {iteration+1}')
                    
                    # Update current pose for next step
                    time.sleep(0.2)
                    updated_ee_data = self.get_ee_pose(arm)
                    if updated_ee_data:
                        current_pose_world = Pose()
                        current_pose_world.position.x = updated_ee_data[0]
                        current_pose_world.position.y = updated_ee_data[1]
                        current_pose_world.position.z = updated_ee_data[2]
                        qx, qy, qz, qw = _euler_to_quaternion(updated_ee_data[3], updated_ee_data[4], updated_ee_data[5])
                        current_pose_world.orientation.x = qx
                        current_pose_world.orientation.y = qy
                        current_pose_world.orientation.z = qz
                        current_pose_world.orientation.w = qw
                        current_pose = self._transform_to_robot_frame(current_pose_world, arm)
                        # CRITICAL: Use actual current orientation (which includes J7 offset) instead of overwriting
                        current_pose.orientation = current_pose_world.orientation  # Keep current orientation (includes J7 offset)
                        
                        # Recalculate remaining distance from current position
                        dx = insert_pose.position.x - current_pose.position.x
                        dy = insert_pose.position.y - current_pose.position.y
                        dz = insert_pose.position.z - current_pose.position.z
                        remaining_distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                    else:
                        # Assume we moved successfully, update remaining distance
                        remaining_distance -= step_distance
                except Exception as step_err:
                    self.get_logger().warn(f'{arm}: Step failed: {step_err}. Trying direct Cartesian path to final target...')
                    # Try Cartesian path to final target as last resort (maintains straight-line movement)
                    try:
                        final_waypoints = [current_pose, insert_pose]
                        final_trajectory = self._compute_cartesian_path(final_waypoints, arm=arm, return_full_trajectory=True)
                        if final_trajectory:
                            self.execute_trajectory_moveit(final_trajectory, arm=arm)
                            self.get_logger().info(f'{arm}: Successfully reached insert pose via direct Cartesian path!')
                            remaining_distance = 0  # Mark as complete
                            break
                        else:
                            self.get_logger().warn(f'{arm}: Direct Cartesian path also failed. Continuing...')
                    except Exception as final_err:
                        self.get_logger().error(f'{arm}: Final IK also failed: {final_err}')
                        return False
            
            # Check final distance and attempt final correction if needed
            time.sleep(0.3)
            final_ee_data = self.get_ee_pose(arm)
            if final_ee_data:
                final_pose_world = Pose()
                final_pose_world.position.x = final_ee_data[0]
                final_pose_world.position.y = final_ee_data[1]
                final_pose_world.position.z = final_ee_data[2]
                final_pose_robot = self._transform_to_robot_frame(final_pose_world, arm)
                final_dx = insert_pose.position.x - final_pose_robot.position.x
                final_dy = insert_pose.position.y - final_pose_robot.position.y
                final_dz = insert_pose.position.z - final_pose_robot.position.z
                final_distance = math.sqrt(final_dx*final_dx + final_dy*final_dy + final_dz*final_dz)
                
                # If close but not quite there, try one more Cartesian path to complete insertion
                if 0.005 < final_distance <= 0.03:  # Between 0.5-3cm away
                    self.get_logger().info(f'{arm}: Close to target (distance: {final_distance*100:.2f}cm). Attempting final Cartesian path to complete insertion...')
                    try:
                        # Use current pose and target for final straight-line movement
                        final_current_pose = self._transform_to_robot_frame(final_pose_world, arm)
                        final_current_pose.orientation = current_pose.orientation  # Keep current orientation
                        final_waypoints = [final_current_pose, insert_pose]
                        final_trajectory = self._compute_cartesian_path(final_waypoints, arm=arm, return_full_trajectory=True)
                        if final_trajectory:
                            self.execute_trajectory_moveit(final_trajectory, arm=arm)
                            time.sleep(0.3)
                            self.get_logger().info(f'{arm}: Final correction completed via Cartesian path!')
                        else:
                            self.get_logger().warn(f'{arm}: Final Cartesian path failed, but insertion may still be complete.')
                    except Exception as final_cart_err:
                        self.get_logger().warn(f'{arm}: Final Cartesian correction failed: {final_cart_err}')
                
                if final_distance <= 0.02:  # Within 2cm is acceptable
                    self.get_logger().info(f'{arm}: Successfully completed insertion! Final distance: {final_distance*100:.2f}cm')
                else:
                    self.get_logger().warn(f'{arm}: Insertion incomplete. Final distance: {final_distance*100:.1f}cm')
            else:
                self.get_logger().warn(f'{arm}: Could not verify final insertion distance (EE pose unavailable)')
            
            self.get_logger().info(f'{arm}: Insert sequence complete! Used frame-based approach for perfect alignment and straight insertion.')
            # joints_insert = self._compute_cartesian_path(waypoints_insert, arm=arm)
            # self.send_joint_trajectory(joints_insert, arm=arm, seconds=3.0)
        except Exception as e:
            self.get_logger().error(f'{arm}: Insertion failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
    
    def _transform_to_robot_frame(self, world_pose: Pose, arm: str) -> Pose:
        """Transform a pose from world frame to robot base frame."""
        robot = self.robot_poses[arm]
        
        # Translate
        dx = world_pose.position.x - robot['x']
        dy = world_pose.position.y - robot['y']
        dz = world_pose.position.z - robot['z']
        
        # Rotate (inverse of robot yaw)
        cos_yaw = math.cos(-robot['yaw'])
        sin_yaw = math.sin(-robot['yaw'])
        
        local_x = dx * cos_yaw - dy * sin_yaw
        local_y = dx * sin_yaw + dy * cos_yaw
        local_z = dz
        
        local_pose = Pose()
        local_pose.position.x = local_x
        local_pose.position.y = local_y
        local_pose.position.z = local_z
        local_pose.orientation = world_pose.orientation
        
        return local_pose
    
    def _add_held_object_as_collision(self, arm: str, object_name: str, object_data: dict) -> None:
        """Add the object held by the other arm as a collision object in the planning scene."""
        from moveit_msgs.msg import PlanningScene, CollisionObject
        from shape_msgs.msg import SolidPrimitive
        
        # DEBUG: Log held objects state
        self.get_logger().info(f'[COLLISION DEBUG] Adding held object collision: arm={arm}, object={object_name}')
        self.get_logger().info(f'[COLLISION DEBUG] Current held_objects state: {self.held_objects}')
        
        # Get the EE pose of the arm holding the object
        ee_data = self.get_ee_pose(arm)
        if not ee_data:
            self.get_logger().error(f'[COLLISION DEBUG] Cannot get EE pose for {arm} to add held object collision!')
            self.get_logger().error(f'[COLLISION DEBUG] This means collision object will NOT be added - collision may occur!')
            return
        
        ee_x, ee_y, ee_z, ee_roll, ee_pitch, ee_yaw = ee_data
        self.get_logger().info(f'[COLLISION DEBUG] {arm} EE pose: X={ee_x:.3f}, Y={ee_y:.3f}, Z={ee_z:.3f}, Roll={ee_roll:.3f}, Pitch={ee_pitch:.3f}, Yaw={ee_yaw:.3f}')
        
        # Create collision object for the held object
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        
        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = f'{object_name}_held_by_{arm}'
        obj.operation = CollisionObject.MOVE  # MOVE to update position dynamically
        
        # Create box primitive with object dimensions
        obj_primitive = SolidPrimitive()
        obj_primitive.type = SolidPrimitive.BOX
        obj_primitive.dimensions = object_data['size']
        
        # Position the object at the EE location
        # The object is held by the gripper, so it's at the EE position
        # Account for orientation if needed (hollow is rotated Pitch=90deg after Insert Prep)
        obj_pose = Pose()
        obj_pose.position.x = ee_x
        obj_pose.position.y = ee_y
        obj_pose.position.z = ee_z
        
        # Use the object's orientation (from Insert Prep: Pitch=90deg for hollow)
        # For hollow objects held by Panda 2, they should be rotated Pitch=90deg
        if 'hollow' in object_name:
            # Hollow is rotated Pitch=90deg (1.5708 rad) to face -X
            qx, qy, qz, qw = _euler_to_quaternion(0.0, 1.5708, 0.0)
            obj_pose.orientation.x = qx
            obj_pose.orientation.y = qy
            obj_pose.orientation.z = qz
            obj_pose.orientation.w = qw
        else:
            # Use identity orientation
            obj_pose.orientation.w = 1.0
        
        obj.primitives = [obj_primitive]
        obj.primitive_poses = [obj_pose]
        
        scene.world.collision_objects.append(obj)
        
        # Publish to planning scene - publish multiple times to ensure MoveIt receives it
        import time
        self.get_logger().info(f'[COLLISION DEBUG] Publishing collision object: ID={obj.id}, Size={obj_primitive.dimensions}, Position=({obj_pose.position.x:.3f}, {obj_pose.position.y:.3f}, {obj_pose.position.z:.3f})')
        for i in range(3):
            self.planning_scene_pub.publish(scene)
            time.sleep(0.05)
        self.get_logger().info(f'[COLLISION DEBUG] ✓ Published {object_name} held by {arm} as collision object at ({ee_x:.3f}, {ee_y:.3f}, {ee_z:.3f}) - MoveIt should now avoid this object')
    
    # ============================================================
    # TODO: Copy remaining methods from dual_arm_gui.py with these changes:
    # 1. Replace namespaced service calls with unified ones
    # 2. Add group_name parameter to MoveIt requests
    # 3. Use prefixed joint names throughout
    # 4. Use unified planning_scene_pub
    # 5. No need to manually publish collision objects for other arm!
    # ============================================================


class DualPandaUnifiedGUI:
    """Tkinter GUI for unified dual Panda control."""
    
    def __init__(self):
        rclpy.init()
        self.node = DualPandaUnifiedNode()
        
        # CRITICAL: Spin node a few times IMMEDIATELY to register subscriptions
        # This ensures /objects_poses_sim subscription is active before background thread starts
        self.node.get_logger().info('[CRITICAL] Doing initial spins to register /objects_poses_sim subscription...')
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().info('[CRITICAL] ✓ Initial spins complete - subscriptions should be registered!')
        
        # Start ROS node in background thread for continuous spinning
        self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self.ros_thread.start()
        
        # Create GUI (similar structure to original)
        self.root = tk.Tk()
        self.root.title("Unified Dual Panda Control")
        self.root.geometry("1200x800")
        self.root.protocol('WM_DELETE_WINDOW', self.shutdown)
        
        # TODO: Build GUI layout (similar to original dual_arm_gui.py)
        self._build_layout()
        
        # Schedule periodic updates
        self._update_live_monitor()
    
    def _ros_spin(self):
        """ROS spin loop in background thread."""
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
    
    def _build_layout(self):
        """Build GUI layout (similar to original)."""
        main_frame = ttk.Frame(self.root, padding=12)
        main_frame.grid(column=0, row=0, sticky='nsew')
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # Title (conditional warning)
        all_services = self.node.services_available.get('ik', False) and \
                       self.node.services_available.get('fk', False) and \
                       self.node.services_available.get('cartesian', False)
        
        if all_services:
            title_text = 'Unified Dual Panda Control'
            title_color = 'green'
        else:
            title_text = 'Unified Dual Panda Control (INCOMPLETE SETUP)'
            title_color = 'orange'
        
        title_label = ttk.Label(main_frame, text=title_text, font=('Arial', 16, 'bold'), foreground=title_color)
        title_label.grid(column=0, row=0, columnspan=3, pady=(0, 10))
        
        # Status/warning row (will be updated when services become available)
        self.status_frame = ttk.Frame(main_frame)
        self.status_frame.grid(column=0, row=1, columnspan=3, pady=(0, 20))
        
        if not all_services:
            warning_text = 'NOTE: MoveIt services not available. Waiting for move_group to start...'
            self.warning_label = ttk.Label(self.status_frame, text=warning_text, 
                                          font=('Arial', 10), foreground='orange')
            self.warning_label.pack()
        else:
            self.status_label = ttk.Label(self.status_frame, text='✓ All MoveIt services available', 
                                         font=('Arial', 10), foreground='green')
            self.status_label.pack()
            self.warning_label = None
        
        # Register callback for when services become available
        self.node._on_services_ready_callback = self._update_service_status
        
        # Check if services are already available (e.g., if GUI started after MoveIt)
        all_services_ready = (self.node.services_available.get('ik', False) and 
                              self.node.services_available.get('fk', False) and 
                              self.node.services_available.get('cartesian', False))
        if all_services_ready:
            # Update immediately if already available
            self.root.after(500, self._update_service_status)

        # Create frames for each arm
        panda1_frame = ttk.LabelFrame(main_frame, text='Panda 1', padding=10)
        panda1_frame.grid(column=0, row=2, padx=10, pady=5, sticky='nsew')
        
        panda2_frame = ttk.LabelFrame(main_frame, text='Panda 2', padding=10)
        panda2_frame.grid(column=1, row=2, padx=10, pady=5, sticky='nsew')
        
        # Monitor Frame
        monitor_frame = ttk.LabelFrame(main_frame, text='Live Monitor', padding=10)
        monitor_frame.grid(column=2, row=2, padx=10, pady=5, sticky='nsew')

        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.columnconfigure(2, weight=1)

        # Initialize status vars
        self.status_var_panda1 = tk.StringVar(value='Ready (Service check pending...)')
        self.status_var_panda2 = tk.StringVar(value='Ready (Service check pending...)')

        # Build Panda 1 controls
        self._build_arm_controls(panda1_frame, 'panda1')
        
        # Build Panda 2 controls
        self._build_arm_controls(panda2_frame, 'panda2')
        
        # Build Monitor
        self._build_monitor(monitor_frame)

        # Global controls
        global_frame = ttk.LabelFrame(main_frame, text='Global Controls', padding=10)
        global_frame.grid(column=0, row=3, columnspan=3, pady=10, sticky='ew')
        
        global_buttons = ttk.Frame(global_frame)
        global_buttons.grid(column=0, row=0, sticky='ew')
        global_buttons.columnconfigure((0, 1, 2), weight=1)
        
        ttk.Button(global_buttons, text='Move Both to Neutral', command=self.move_both_to_neutral).grid(column=0, row=0, padx=4)
        ttk.Button(global_buttons, text='Open Both Grippers', command=lambda: self._send_gripper_both(0.04)).grid(column=1, row=0, padx=4)
        ttk.Button(global_buttons, text='Close Both Grippers', command=lambda: self._send_gripper_both(0.01)).grid(column=2, row=0, padx=4)
        
        # Dual Arm Tasks
        task_frame = ttk.LabelFrame(main_frame, text='Dual Arm Tasks (Insertion)', padding=10)
        task_frame.grid(column=0, row=4, columnspan=3, pady=10, sticky='ew')
        
        # Panda 2 (Holder)
        ttk.Label(task_frame, text='Panda 2 (Holder):').grid(column=0, row=0, sticky='w')
        self.holder_obj_var = tk.StringVar(value='red_hollow')
        holder_opts = ['red_hollow', 'green_hollow']
        ttk.Combobox(task_frame, textvariable=self.holder_obj_var, values=holder_opts, state='readonly', width=12).grid(column=1, row=0, padx=4)
        ttk.Button(task_frame, text='Insert Prep', command=self._insert_prep).grid(column=2, row=0, padx=4)
        
        # Panda 1 (Inserter)
        ttk.Label(task_frame, text='Panda 1 (Inserter):').grid(column=3, row=0, sticky='w', padx=(20, 0))
        self.inserter_obj_var = tk.StringVar(value='red_solid')
        inserter_opts = ['red_solid', 'green_solid']
        ttk.Combobox(task_frame, textvariable=self.inserter_obj_var, values=inserter_opts, state='readonly', width=12).grid(column=4, row=0, padx=4)
        ttk.Button(task_frame, text='Insert', command=self._do_insertion).grid(column=5, row=0, padx=4)
    
    def _build_arm_controls(self, parent: ttk.Frame, arm: str) -> None:
        """Build control widgets for one arm."""
        # Position controls
        ttk.Label(parent, text='Target Position (meters)').grid(column=0, row=0, columnspan=3, pady=(0, 4))
        entry_x = self._make_entry(parent, 'X', 0.5, column=0, row=1)
        entry_y = self._make_entry(parent, 'Y', 0.0, column=1, row=1)
        entry_z = self._make_entry(parent, 'Z', 0.4, column=2, row=1)

        # Orientation controls
        ttk.Label(parent, text='Target Orientation (radians)').grid(column=0, row=2, columnspan=3, pady=(12, 4))
        entry_roll = self._make_entry(parent, 'Roll', math.pi, column=0, row=3)
        entry_pitch = self._make_entry(parent, 'Pitch', 0.0, column=1, row=3)
        entry_yaw = self._make_entry(parent, 'Yaw', 0.0, column=2, row=3)

        # Store references
        if arm == 'panda1':
            self.entry_x_p1, self.entry_y_p1, self.entry_z_p1 = entry_x, entry_y, entry_z
            self.entry_roll_p1, self.entry_pitch_p1, self.entry_yaw_p1 = entry_roll, entry_pitch, entry_yaw
        else:
            self.entry_x_p2, self.entry_y_p2, self.entry_z_p2 = entry_x, entry_y, entry_z
            self.entry_roll_p2, self.entry_pitch_p2, self.entry_yaw_p2 = entry_roll, entry_pitch, entry_yaw

        # Buttons
        buttons = ttk.Frame(parent)
        buttons.grid(column=0, row=4, columnspan=3, pady=(16, 8), sticky='ew')
        buttons.columnconfigure((0, 1, 2), weight=1)
        
        move_cmd = lambda: self.move_to_pose(arm)
        plan_cmd = lambda: self.move_to_pose(arm, execute=False)
        neutral_cmd = lambda: self.move_to_neutral(arm)
        
        ttk.Button(buttons, text='Move', command=move_cmd).grid(column=0, row=0, padx=4)
        ttk.Button(buttons, text='Plan Only', command=plan_cmd).grid(column=1, row=0, padx=4)
        ttk.Button(buttons, text='Neutral', command=neutral_cmd).grid(column=2, row=0, padx=4)

        # Gripper controls
        grip = ttk.Frame(parent)
        grip.grid(column=0, row=5, columnspan=3, sticky='ew')
        grip.columnconfigure((0, 1), weight=1)
        
        open_cmd = lambda: self._send_gripper(0.04, arm)
        close_cmd = lambda: self._send_gripper(0.01, arm)
        
        ttk.Button(grip, text='Open Gripper', command=open_cmd).grid(column=0, row=0, padx=4, pady=(0, 4))
        ttk.Button(grip, text='Close Gripper', command=close_cmd).grid(column=1, row=0, padx=4, pady=(0, 4))
        
        # Object pick controls
        pick_frame = ttk.Frame(parent)
        pick_frame.grid(column=0, row=6, columnspan=3, pady=(12, 4), sticky='ew')
        pick_frame.columnconfigure(0, weight=1)
        
        ttk.Label(pick_frame, text='Pick Object:').grid(column=0, row=0, sticky='w', padx=(0, 4))
        
        # Dropdown for object selection
        object_var = tk.StringVar(value='None')
        object_names = ['None', 'red_block', 'green_block', 'red_solid', 'green_solid', 'red_hollow', 'green_hollow']
        object_dropdown = ttk.Combobox(pick_frame, textvariable=object_var, values=object_names, state='readonly', width=15)
        object_dropdown.grid(column=1, row=0, padx=4, sticky='ew')
        
        # Pick button
        pick_cmd = lambda: self._pick_object(object_var.get(), arm)
        ttk.Button(pick_frame, text='Pick', command=pick_cmd).grid(column=2, row=0, padx=4)
        
        # Store references
        if arm == 'panda1':
            self.object_var_p1 = object_var
            self.object_dropdown_p1 = object_dropdown
        else:
            self.object_var_p2 = object_var
            self.object_dropdown_p2 = object_dropdown

        # Place controls
        place_frame = ttk.Frame(parent)
        place_frame.grid(column=0, row=7, columnspan=3, pady=(4, 4), sticky='ew')
        place_frame.columnconfigure(0, weight=1)
        
        ttk.Label(place_frame, text='Place Location (Surface):').grid(column=0, row=0, columnspan=3, sticky='w')
        
        # Simple X, Y, Z entries for place
        place_coords_frame = ttk.Frame(place_frame)
        place_coords_frame.grid(column=0, row=1, columnspan=3, sticky='ew')
        place_coords_frame.columnconfigure((0, 1, 2), weight=1)
        
        # Defaults based on arm
        def_x = 0.5
        def_y = 0.3 if arm == 'panda1' else -0.3
        def_z = 0.2
        
        def make_mini(p, l, d, c):
            f = ttk.Frame(p)
            f.grid(column=c, row=0, padx=2)
            ttk.Label(f, text=l).pack(side='left')
            e = ttk.Entry(f, width=6)
            e.insert(0, str(d))
            e.pack(side='left')
            return e
            
        px = make_mini(place_coords_frame, 'X:', def_x, 0)
        py = make_mini(place_coords_frame, 'Y:', def_y, 1)
        pz = make_mini(place_coords_frame, 'Z:', def_z, 2)
        
        place_cmd = lambda: self._place_object(arm, px.get(), py.get(), pz.get())
        ttk.Button(place_frame, text='Place', command=place_cmd).grid(column=0, row=2, columnspan=3, pady=4)

        # Status
        status_var = self.status_var_panda1 if arm == 'panda1' else self.status_var_panda2
        ttk.Label(parent, textvariable=status_var, foreground='gray').grid(column=0, row=8, columnspan=3, pady=(12, 0), sticky='w')

    def _make_entry(self, frame: ttk.Frame, label: str, default: float, column: int, row: int) -> tk.Entry:
        """Create a labeled entry widget."""
        container = ttk.Frame(frame)
        container.grid(column=column, row=row, padx=4, pady=2, sticky='ew')
        container.columnconfigure(1, weight=1)
        ttk.Label(container, text=label).grid(column=0, row=0, sticky='w')
        entry = ttk.Entry(container, width=10)
        entry.insert(0, f'{default:.3f}')
        entry.grid(column=1, row=0, sticky='ew')
        return entry

    def _build_monitor(self, parent: ttk.Frame) -> None:
        """Build live monitor widgets."""
        self.monitor_vars = {}
        
        # Objects only (including blue_block for TAMP)
        objects = ['red_block', 'green_block', 'red_solid', 'green_solid', 'red_hollow', 'green_hollow', 'blue_block']
        
        row = 0
        for obj in objects:
            container = ttk.Frame(parent)
            container.grid(column=0, row=row, sticky='ew', pady=1)
            
            lbl = ttk.Label(container, text=f'{obj}:', font=('Arial', 9, 'bold'), width=12)
            lbl.pack(side='left')
            
            var = tk.StringVar(value='Waiting...')
            val_lbl = ttk.Label(container, textvariable=var, font=('Courier', 9))
            val_lbl.pack(side='left')
            
            self.monitor_vars[obj] = var
            row += 1
            
        # Separator
        ttk.Separator(parent, orient='horizontal').grid(column=0, row=row, sticky='ew', pady=5)
        row += 1
        
        # EE Poses
        ttk.Label(parent, text='End Effector Poses (Live):', font=('Arial', 10, 'bold')).grid(column=0, row=row, sticky='w', pady=2)
        row += 1
        
        self.ee_vars = {}
        for arm in ['panda1', 'panda2']:
            container = ttk.Frame(parent)
            container.grid(column=0, row=row, sticky='ew', pady=1)
            
            lbl = ttk.Label(container, text=f'{arm}:', font=('Arial', 9, 'bold'), width=12)
            lbl.pack(side='left')
            
            var = tk.StringVar(value='No Data')
            val_lbl = ttk.Label(container, textvariable=var, font=('Courier', 9))
            val_lbl.pack(side='left')
            
            self.ee_vars[arm] = var
            row += 1
    
    def _update_live_monitor(self):
        """Update live pose monitor (similar to original)."""
        # TODO: Implement full logic when node methods are ready
        try:
            # Try to update objects (will fail if node methods not implemented)
            for obj_name, var in self.monitor_vars.items():
                try:
                    data = self.node.get_pose(obj_name)
                    if data:
                        pos = data.get('position', [0, 0, 0])
                        yaw = data.get('yaw', 0)
                        text = f"X:{pos[0]:.2f} Y:{pos[1]:.2f} Z:{pos[2]:.2f} Yaw:{yaw:.2f}"
                        var.set(text)
                    else:
                        var.set("No Data")
                except:
                    var.set("Waiting...")
            
            # Try to update EE poses
            for arm, var in self.ee_vars.items():
                try:
                    pose = self.node.get_ee_pose(arm)
                    if pose:
                        text = f"X:{pose[0]:.2f} Y:{pose[1]:.2f} Z:{pose[2]:.2f} RPY:{pose[3]:.1f},{pose[4]:.1f},{pose[5]:.1f}"
                        var.set(text)
                    else:
                        var.set("No Data")
                except:
                    var.set("No Data")
        except:
            pass
        
        self.root.after(100, self._update_live_monitor)
    
    def _update_service_status(self):
        """Update GUI status when MoveIt services become available."""
        def update_gui():
            # Clear the warning label
            for widget in self.status_frame.winfo_children():
                widget.destroy()
            
            # Add success status label
            self.status_label = ttk.Label(self.status_frame, text='✓ All MoveIt services available', 
                                         font=('Arial', 10), foreground='green')
            self.status_label.pack()
            self.warning_label = None
            
            # Update status variables to show "Ready"
            self.status_var_panda1.set('Ready ✓')
            self.status_var_panda2.set('Ready ✓')
            
            # Update title to green
            # Find and update the title label
            try:
                for widget in self.root.winfo_children():
                    if isinstance(widget, ttk.Frame):
                        for child in widget.winfo_children():
                            if isinstance(child, ttk.Label) and 'Unified Dual Panda Control' in str(child.cget('text')):
                                child.config(text='Unified Dual Panda Control', foreground='green')
                                break
            except Exception as e:
                print(f"Error updating title: {e}")
        
        # Schedule GUI update in main thread (Tkinter is not thread-safe)
        self.root.after(0, update_gui)
    
    def _read_pose(self, arm: str) -> Pose:
        """Read pose from GUI entries for specified arm."""
        if arm == 'panda1':
            entries = (self.entry_x_p1, self.entry_y_p1, self.entry_z_p1,
                      self.entry_roll_p1, self.entry_pitch_p1, self.entry_yaw_p1)
        else:
            entries = (self.entry_x_p2, self.entry_y_p2, self.entry_z_p2,
                      self.entry_roll_p2, self.entry_pitch_p2, self.entry_yaw_p2)

        try:
            x = float(entries[0].get())
            y = float(entries[1].get())
            z = float(entries[2].get())
            roll = float(entries[3].get())
            pitch = float(entries[4].get())
            yaw = float(entries[5].get())
        except ValueError as exc:
            raise ValueError('Inputs must be numeric.') from exc

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        
        # Convert Euler to quaternion
        qx, qy, qz, qw = _euler_to_quaternion(roll, pitch, yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        
        return pose
    
    def move_to_pose(self, arm: str, execute: bool = True) -> None:
        """Move specified arm to target pose."""
        status_var = self.status_var_panda1 if arm == 'panda1' else self.status_var_panda2
        all_services = (self.node.services_available.get('ik', False) and 
                        self.node.services_available.get('fk', False) and 
                        self.node.services_available.get('cartesian', False))
        if not all_services:
            status_var.set('Services not available (Waiting for MoveIt...)')
            return
        
        try:
            pose = self._read_pose(arm)
        except ValueError as exc:
            messagebox.showerror('Invalid input', str(exc))
            return

        status_var.set('Computing IK...')
        self.root.update_idletasks()

        try:
            # Pass world pose directly (MoveIt planning frame is 'world')
            joint_positions = self.node.compute_ik(pose, arm=arm, base_frame='world')
        except RuntimeError as exc:
            status_var.set(f'IK failed: {exc}')
            return

        status_var.set('IK success. ' + ('Executing...' if execute else 'Not executing.'))
        self.root.update_idletasks()

        if execute:
            success = self.node.send_joint_trajectory(joint_positions, arm=arm)
            status_var.set('Command sent.' if success else 'Trajectory execution failed.')
        else:
            formatted = ', '.join(f'{val:.2f}' for val in joint_positions)
            status_var.set(f'Joint solution: [{formatted}]')
    
    def move_to_neutral(self, arm: str) -> None:
        """Move specified arm to neutral pose."""
        status_var = self.status_var_panda1 if arm == 'panda1' else self.status_var_panda2
        all_services = (self.node.services_available.get('ik', False) and 
                        self.node.services_available.get('fk', False) and 
                        self.node.services_available.get('cartesian', False))
        if not all_services:
            status_var.set('Services not available (Waiting for MoveIt...)')
            return
        
        status_var.set('Moving to neutral...')
        success = self.node.send_joint_trajectory(self.node.neutral_pose, arm=arm)
        status_var.set('Neutral command sent.' if success else 'Neutral command failed.')
    
    def move_both_to_neutral(self) -> None:
        """Move both arms to neutral pose."""
        self.move_to_neutral('panda1')
        self.move_to_neutral('panda2')
    
    def _send_gripper(self, width: float, arm: str) -> None:
        """Send gripper command to specified arm."""
        status_var = self.status_var_panda1 if arm == 'panda1' else self.status_var_panda2
        status_var.set('Sending gripper command...')
        self.node.send_gripper_goal(width, arm=arm)
        status_var.set('Gripper command sent.')
    
    def _send_gripper_both(self, width: float) -> None:
        """Send gripper command to both arms."""
        self._send_gripper(width, 'panda1')
        self._send_gripper(width, 'panda2')
    
    def _pick_object(self, object_name: str, arm: str) -> None:
        """Handle pick object button click."""
        if object_name == 'None':
            messagebox.showwarning('No Object', 'Please select an object to pick.')
            return
        
        status_var = self.status_var_panda1 if arm == 'panda1' else self.status_var_panda2
        status_var.set(f'Picking {object_name}...')
        self.root.update_idletasks()
        
        # Run pick in separate thread to avoid blocking GUI
        def pick_thread():
            try:
                success = self.node.pick_object(object_name, arm=arm)
                if success:
                    status_var.set(f'Pick {object_name} complete!')
                else:
                    status_var.set(f'Pick {object_name} failed!')
            except Exception as e:
                status_var.set(f'Pick error: {str(e)}')
                self.node.get_logger().error(f'Pick error: {e}')
        
        threading.Thread(target=pick_thread, daemon=True).start()

    def _place_object(self, arm: str, x_str: str, y_str: str, z_str: str) -> None:
        """Handle place object button click."""
        status_var = self.status_var_panda1 if arm == 'panda1' else self.status_var_panda2
        
        try:
            x = float(x_str)
            y = float(y_str)
            z = float(z_str)
        except ValueError:
            messagebox.showerror('Invalid input', 'Place coordinates must be numeric.')
            return

        status_var.set(f'Placing at ({x}, {y}, {z})...')
        self.root.update_idletasks()

        def place_thread():
            try:
                success = self.node.place_object(arm, [x, y, z])
                if success:
                    status_var.set('Place complete!')
                else:
                    status_var.set('Place failed!')
            except Exception as e:
                status_var.set(f'Place error: {str(e)}')
                self.node.get_logger().error(f'Place error: {e}')

        threading.Thread(target=place_thread, daemon=True).start()

    def _insert_prep(self) -> None:
        """Handle Insert Prep button."""
        obj = self.holder_obj_var.get()
        self.status_var_panda2.set(f'Prepping {obj}...')
        
        def task():
            if self.node.prepare_for_insertion(obj, arm='panda2'):
                self.status_var_panda2.set('Prep Complete. Holding.')
            else:
                self.status_var_panda2.set('Prep Failed.')
        
        threading.Thread(target=task, daemon=True).start()

    def _do_insertion(self) -> None:
        """Handle Insert button."""
        obj = self.inserter_obj_var.get()
        self.status_var_panda1.set(f'Inserting {obj}...')
        
        def task():
            if self.node.perform_insertion(obj, arm='panda1'):
                self.status_var_panda1.set('Insertion Complete.')
            else:
                self.status_var_panda1.set('Insertion Failed.')
        
        threading.Thread(target=task, daemon=True).start()
    
    def shutdown(self):
        """Clean shutdown."""
        self.status_var_panda1.set('Closing...')
        self.status_var_panda2.set('Closing...')
        self.root.update_idletasks()
        
        # Destroy node before shutting down rclpy
        try:
            self.node.destroy_node()
        except Exception:
            pass
        
        # Shutdown rclpy
        try:
            rclpy.shutdown()
        except Exception:
            pass
        
        self.root.destroy()
    
    def run(self):
        """Start GUI main loop."""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert roll-pitch-yaw to quaternion."""
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


def _quaternion_multiply(q1: Tuple[float, float, float, float], q2: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """Multiply two quaternions: q_result = q1 * q2."""
    w1, x1, y1, z1 = q1[3], q1[0], q1[1], q1[2]
    w2, x2, y2, z2 = q2[3], q2[0], q2[1], q2[2]
    
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    
    return x, y, z, w


def _quaternion_rotate_vector(q: Tuple[float, float, float, float], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Rotate a vector by a quaternion: v_rotated = q * v * q^-1."""
    x, y, z, w = q[0], q[1], q[2], q[3]
    vx, vy, vz = v[0], v[1], v[2]
    
    # Convert vector to quaternion (pure quaternion: w=0)
    v_quat = (vx, vy, vz, 0.0)
    q_inv = (-x, -y, -z, w)  # Quaternion inverse (conjugate for unit quaternion)
    
    # Rotate: q * v * q^-1
    temp = _quaternion_multiply((x, y, z, w), v_quat)
    result = _quaternion_multiply(temp, q_inv)
    
    return (result[0], result[1], result[2])


def _compose_transforms(pose1: Pose, pose2: Pose) -> Pose:
    """
    Compose two transforms: result = pose1 * pose2
    
    Given:
    - pose1: Transform from frame A to frame B
    - pose2: Transform from frame B to frame C
    
    Returns:
    - result: Transform from frame A to frame C
    """
    result = Pose()
    
    # Extract quaternions
    q1 = (pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w)
    q2 = (pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w)
    
    # Compose orientations: q_result = q1 * q2
    q_result = _quaternion_multiply(q1, q2)
    result.orientation.x = q_result[0]
    result.orientation.y = q_result[1]
    result.orientation.z = q_result[2]
    result.orientation.w = q_result[3]
    
    # Compose positions: p_result = p1 + R1 * p2
    p1 = (pose1.position.x, pose1.position.y, pose1.position.z)
    p2 = (pose2.position.x, pose2.position.y, pose2.position.z)
    
    # Rotate p2 by q1
    p2_rotated = _quaternion_rotate_vector(q1, p2)
    
    # Add rotated p2 to p1
    result.position.x = p1[0] + p2_rotated[0]
    result.position.y = p1[1] + p2_rotated[1]
    result.position.z = p1[2] + p2_rotated[2]
    
    return result


def _apply_transform(transform: Pose, point: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """
    Apply a transform to a point: result_point = transform * point
    
    Args:
        transform: Pose representing the transform
        point: (x, y, z) tuple
    
    Returns:
        Transformed point as (x, y, z) tuple
    """
    # Rotate the point
    q = (transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w)
    rotated_point = _quaternion_rotate_vector(q, point)
    
    # Translate
    result = (
        transform.position.x + rotated_point[0],
        transform.position.y + rotated_point[1],
        transform.position.z + rotated_point[2]
    )
    
    return result


def _pose_from_position_quaternion(pos: Tuple[float, float, float], quat: Tuple[float, float, float, float]) -> Pose:
    """Create a Pose from position and quaternion."""
    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.position.z = pos[2]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose


def _invert_transform(transform: Pose) -> Pose:
    """
    Invert a transform: result = transform^-1
    
    For a transform T = [R, p], the inverse is T^-1 = [R^T, -R^T * p]
    """
    result = Pose()
    
    # Invert quaternion (conjugate for unit quaternion)
    q = (transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w)
    q_inv = (-q[0], -q[1], -q[2], q[3])
    result.orientation.x = q_inv[0]
    result.orientation.y = q_inv[1]
    result.orientation.z = q_inv[2]
    result.orientation.w = q_inv[3]
    
    # Invert position: -R^T * p
    p = (transform.position.x, transform.position.y, transform.position.z)
    p_rotated = _quaternion_rotate_vector(q_inv, p)
    result.position.x = -p_rotated[0]
    result.position.y = -p_rotated[1]
    result.position.z = -p_rotated[2]
    
    return result


def main():
    """Main entry point."""
    gui = DualPandaUnifiedGUI()
    try:
        gui.run()
    except KeyboardInterrupt:
        gui.shutdown()


if __name__ == '__main__':
    main()

