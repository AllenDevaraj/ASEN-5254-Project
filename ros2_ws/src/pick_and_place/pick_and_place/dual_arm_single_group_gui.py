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
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


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
            'cartesian': False
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
            'red_solid': {'position': [0.6, -0.2, 0.25], 'size': [0.03, 0.03, 0.08], 'yaw': 0.0},
            'green_solid': {'position': [0.6, 0.2, 0.25], 'size': [0.03, 0.03, 0.08], 'yaw': 0.0},
            'red_hollow': {'position': [0.9, -0.2, 0.231], 'size': [0.08, 0.05, 0.05], 'yaw': -1.5708},
            'green_hollow': {'position': [0.9, 0.2, 0.231], 'size': [0.08, 0.05, 0.05], 'yaw': -1.5708}
        }
        
        self.objects_lock = threading.Lock()
        self.held_objects = {'panda1': None, 'panda2': None}
        self.collision_object_ids = {'panda1': set(), 'panda2': set()}  # Track per arm
        
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
    
    def _setup_planning_scene(self):
        """Set up planning scene with table and objects. Adapted for unified setup using 'world' frame."""
        self.get_logger().info('Setting up planning scene...')
        
        with self.objects_lock:
            objects_copy = {k: v.copy() for k, v in self.objects.items()}
        
        # Create planning scene message (unified - single scene for both arms)
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = []
        scene.robot_state.is_diff = True
        
        # Add table (WORLD FRAME)
        table_obj = CollisionObject()
        table_obj.header.frame_id = 'world'
        table_obj.header.stamp = self.get_clock().now().to_msg()
        table_obj.id = 'table'
        table_obj.operation = CollisionObject.ADD  # Table is static, use ADD
        
        table_primitive = SolidPrimitive()
        table_primitive.type = SolidPrimitive.BOX
        table_primitive.dimensions = self.table_size
        
        table_obj.primitives = [table_primitive]
        table_obj.primitive_poses = [self.table_pose]
        scene.world.collision_objects.append(table_obj)
        
        # Add all objects (WORLD FRAME) - Use MOVE operation for dynamic updates
        # First time: ADD, subsequent: MOVE (to update positions)
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
            
            # Create world pose
            world_pose = Pose()
            world_pose.position.x = obj_data['position'][0]
            world_pose.position.y = obj_data['position'][1]
            world_pose.position.z = obj_data['position'][2]
            
            # Set orientation from yaw
            qx, qy, qz, qw = _euler_to_quaternion(0.0, 0.0, obj_data['yaw'])
            world_pose.orientation.x = qx
            world_pose.orientation.y = qy
            world_pose.orientation.z = qz
            world_pose.orientation.w = qw
            
            obj.primitives = [obj_primitive]
            obj.primitive_poses = [world_pose]
            scene.world.collision_objects.append(obj)
        
        # Publish scene (unified publisher) - publish multiple times to ensure MoveIt receives it
        import time
        for i in range(3):
            self.planning_scene_pub.publish(scene)
            time.sleep(0.1)
        self.get_logger().info(f'Planning scene published (unified) - {len(scene.world.collision_objects)} collision objects (table + {len(scene.world.collision_objects) - 1} objects)')
        
        # Wait a bit for scene to be processed by MoveIt
        time.sleep(0.5)
    
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

    def _compute_cartesian_path(self, waypoints: List[Pose], arm: str = 'panda1') -> List[float]:
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
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        
        # CRITICAL: Provide FULL unified joint state (both arms) so MoveIt can automatically
        # check collisions between both arms and objects. This is the whole point of unified MoveIt!
        if self.joint_state:
            # Include ALL joints from unified joint_state (both panda1_* and panda2_* joints)
            request.start_state.joint_state = self.joint_state
            # Also set the robot state flag to indicate this is a complete state (not a diff)
            request.start_state.is_diff = False  # Complete state for proper collision checking
        
        future = cartesian_client.call_async(request)
        if not self._wait_for_future(future, timeout_sec=5.0):
            self.get_logger().warn('Cartesian path service timed out.')
            return None
        response = future.result()
        
        if not response:
            raise RuntimeError('Cartesian path service call failed.')
        
        if response.fraction < 1.0:
            raise RuntimeError(f'Cartesian path incomplete: {response.fraction * 100:.1f}% feasible')
        
        # Extract joint positions from trajectory
        if not response.solution.joint_trajectory.points:
            raise RuntimeError('No trajectory points in cartesian path response')
        
        # Return joint positions from last waypoint
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

    def _wait_for_future(self, future, timeout_sec=None):
        """Wait for future to complete without spinning the node."""
        import time
        start_time = time.time()
        while not future.done():
            if timeout_sec is not None and (time.time() - start_time) > timeout_sec:
                return False
            time.sleep(0.01)
        return True
    
    def pick_object(self, object_name: str, arm: str = 'panda1') -> bool:
        """
        Pick an object using 5-step pipeline.
        Adapted for unified setup: uses 'world' frame and prefixed link names.
        """
        import time
        
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
            
            # Step 1: Open gripper FIRST
            self.get_logger().info(f'{arm}: Opening gripper...')
            self.send_gripper_goal(0.04, arm=arm)
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
            target_yaw_local += (math.pi / 4.0)  # 45 deg offset
            target_yaw_local += (math.pi / 2.0)  # 90 deg for solids
            
            qx, qy, qz, qw = _euler_to_quaternion(math.pi, 0.0, target_yaw_local)
            pre_grasp_pose.orientation.x = qx
            pre_grasp_pose.orientation.y = qy
            pre_grasp_pose.orientation.z = qz
            pre_grasp_pose.orientation.w = qw
            
            # Move to pre-grasp pose using IK (robot base frame)
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
            target_yaw_local += (math.pi / 4.0)
            target_yaw_local += (math.pi / 2.0)
            
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
            TABLE_TOP_Z = 0.2
            grasp_target_z_world = TABLE_TOP_Z + (obj_size[2] / 2.0)
            
            world_center_pose = Pose()
            world_center_pose.position.x = obj_world_pos[0]
            world_center_pose.position.y = obj_world_pos[1]
            world_center_pose.position.z = grasp_target_z_world
            
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
            
            self.get_logger().info(f'{arm}: Adjusting hover & descending...')
            waypoints = [new_pre_grasp_pose, grasp_pose]  # Robot frame poses (fixes Cartesian path)
            grasp_joints = self._compute_cartesian_path(waypoints, arm=arm)
            self.send_joint_trajectory(grasp_joints, arm=arm, seconds=3.0)
            
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
            lift_joints = self._compute_cartesian_path(lift_waypoints, arm=arm)
            self.send_joint_trajectory(lift_joints, arm=arm, seconds=4.0)  # Longer time for multi-waypoint path
            
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
            
            self.get_logger().info(f'{arm}: Moving to pre-place...')
            # Transform to robot base frame
            pre_place_pose = self._transform_to_robot_frame(world_pre_place, arm)
            pre_place_pose.orientation = world_pre_place.orientation
            pre_place_joints = self.compute_ik(pre_place_pose, arm=arm)  # Uses robot base frame by default
            self.send_joint_trajectory(pre_place_joints, arm=arm, seconds=4.0)
            
            # 2. Descent to Place - WORLD FRAME
            self.get_logger().info(f'{arm}: Descending to place...')
            waypoints = [world_pre_place, world_target]
            place_joints = self._compute_cartesian_path(waypoints, arm=arm)
            self.send_joint_trajectory(place_joints, arm=arm, seconds=3.0)
            
            # 3. Open Gripper
            self.get_logger().info(f'{arm}: Releasing object...')
            self.send_gripper_goal(0.04, arm=arm)
            time.sleep(1.0)
            
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
            
            # 5. Retreat - WORLD FRAME
            world_retreat = Pose()
            world_retreat.position.x = world_target.position.x
            world_retreat.position.y = world_target.position.y
            world_retreat.position.z = world_target.position.z + 0.15
            world_retreat.orientation = world_target.orientation
            
            self.get_logger().info(f'{arm}: Retreating...')
            waypoints_ret = [world_target, world_retreat]
            ret_joints = self._compute_cartesian_path(waypoints_ret, arm=arm)
            self.send_joint_trajectory(ret_joints, arm=arm, seconds=2.0)
            
            return True

        except Exception as e:
            self.get_logger().error(f'{arm}: Place failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
    
    def prepare_for_insertion(self, object_name: str, arm: str = 'panda2') -> bool:
        """Pick up object and move to assembly pose with 90deg rotation. Adapted for unified setup."""
        import time  # Import time for sleep calls
        
        # 1. Pick object
        if not self.pick_object(object_name, arm=arm):
            return False
        
        # 1.5: Apply 45-degree J7 offset right after picking (User Request)
        self.get_logger().info(f'{arm}: Applying 45-degree J7 offset after pick...')
        try:
            # Get current joint positions from the lift trajectory
            # We need to get the current joint state and apply J7 offset
            if self.joint_state:
                # Extract current joints for this arm
                current_joints = []
                for joint_name in self.joint_names[arm]:
                    try:
                        idx = self.joint_state.name.index(joint_name)
                        current_joints.append(self.joint_state.position[idx])
                    except ValueError:
                        current_joints.append(0.0)
                
                # Apply 45-degree offset to J7 (index 6)
                offset_joints = list(current_joints)
                offset_joints[6] += (math.pi / 4.0)
                self.send_joint_trajectory(offset_joints, arm=arm, seconds=1.0)
                time.sleep(0.5)  # Small delay to ensure it completes
            else:
                self.get_logger().warn(f'{arm}: No joint state available for J7 offset, skipping...')
        except Exception as e:
            self.get_logger().warn(f'{arm}: Failed to apply J7 offset: {e}, continuing anyway...')
            
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
        
        self.get_logger().info(f'{arm}: Moving to Assembly Pose (Hole facing -X)...')
        try:
            joints = self.compute_ik(assembly_pose, arm=arm)  # Uses robot base frame by default
            self.send_joint_trajectory(joints, arm=arm, seconds=4.0)
            
            # Step 3: Rotate J7 by 45 degrees (additional rotation after reaching assembly)
            self.get_logger().info(f'{arm}: Rotating J7 by additional 45 degrees...')
            new_joints = list(joints)
            new_joints[6] += (math.pi / 4.0)
            self.send_joint_trajectory(new_joints, arm=arm, seconds=1.0)
            
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

    def perform_insertion(self, object_name: str, arm: str = 'panda1') -> bool:
        """Pick object and insert into the hollow object held by Panda 2. Adapted for unified setup."""
        import time  # Import time at function level since pick_object also imports it
        
        # 1. Pick object
        if not self.pick_object(object_name, arm=arm):
            return False
        
        # 2. Retreat to Safe Position - Transform to robot base frame
        self.get_logger().info(f'{arm}: Retreating to safe position (away from Panda 2)...')
        
        world_safe_retreat_pose = Pose()
        world_safe_retreat_pose.position.x = 0.3
        world_safe_retreat_pose.position.y = 0.15
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
        solid_length, solid_width, solid_height = solid_size[0], solid_size[1], solid_size[2]
        
        self.get_logger().info(f'{arm}: Hollow object LIVE position: X={hx:.3f}, Y={hy:.3f}, Z={hz:.3f}')
        
        # Pre-insert position: 25cm in front of hollow along X-axis
        # Hollow is at x=hx, so 25cm in front (toward Panda 1) is: hx - 0.25
        pre_insert_x = hx - 0.25  # 25cm in front of hollow
        pre_insert_y = hy  # Same Y as hollow
        pre_insert_z = hz  # Same Z as hollow
        
        self.get_logger().info(f'{arm}: Pre-insert target: X={pre_insert_x:.3f} (25cm in front of hollow X={hx:.3f}), Y={pre_insert_y:.3f}, Z={pre_insert_z:.3f}')
        
        # Calculate where gripper TCP needs to be for solid object (upright, held horizontally)
        # Hollow center is at hz (from Gazebo pose, which is the object center)
        # Solid is upright, and when held by horizontal gripper, solid center = gripper TCP position
        # To align solid center with hollow center: gripper_z = hz (hollow center Z)
        # NO offset needed because both are centers
        
        world_pre_insert_pose = Pose()
        world_pre_insert_pose.position.x = pre_insert_x  # 25cm in front of hollow
        world_pre_insert_pose.position.y = pre_insert_y  # Aligned with hollow Y
        world_pre_insert_pose.position.z = hz  # Align solid center with hollow center Z (both are centers)
        
        # Orientation: Point gripper horizontally along X-axis (toward hole)
        # Pitch=90deg (gripper horizontal), Yaw=0deg (pointing straight along X-axis)
        qx, qy, qz, qw = _euler_to_quaternion(0.0, 1.5708, 0.0)  # Pitch=90deg, Yaw=0deg (straight toward hole)
        world_pre_insert_pose.orientation.x = qx
        world_pre_insert_pose.orientation.y = qy
        world_pre_insert_pose.orientation.z = qz
        world_pre_insert_pose.orientation.w = qw
        
        # Transform to robot base frame
        pre_insert_pose = self._transform_to_robot_frame(world_pre_insert_pose, arm)
        pre_insert_pose.orientation = world_pre_insert_pose.orientation
        
        # 5. Insert Target Pose: Move forward 2cm from pre-insert position
        # Pre-insert is at: hx - 0.25
        # Insert should be at: (hx - 0.25) + 0.02 = hx - 0.23 (2cm forward from pre-insert)
        world_insert_pose = Pose()
        world_insert_pose.position.x = pre_insert_x + 0.02  # 2cm forward from pre-insert (along X-axis)
        world_insert_pose.position.y = pre_insert_y  # Same Y as pre-insert
        world_insert_pose.position.z = world_pre_insert_pose.position.z  # Same Z as pre-insert
        world_insert_pose.orientation = world_pre_insert_pose.orientation  # Same orientation
        
        self.get_logger().info(f'{arm}: Insert target (2cm forward): X={world_insert_pose.position.x:.3f}, Y={world_insert_pose.position.y:.3f}, Z={world_insert_pose.position.z:.3f}')
        
        # Transform to robot base frame
        insert_pose = self._transform_to_robot_frame(world_insert_pose, arm)
        insert_pose.orientation = world_insert_pose.orientation
        
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
            self._add_held_object_as_collision(other_arm, hollow_name, hollow_data)
            time.sleep(0.2)  # Wait for planning scene to be processed
            
            # Step 2a: Move to safe position (already transformed above)
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
            
            self._setup_planning_scene()  # Refresh all objects with latest poses
            self._add_held_object_as_collision(other_arm, hollow_name, hollow_data)  # Update held object
            time.sleep(0.5)  # Wait for planning scene to be processed by MoveIt
            
            self.get_logger().info(f'{arm}: Planning collision-free path around Panda 2 and hollow object...')
            self.get_logger().info(f'{arm}: Using unified joint state (both arms) for collision checking...')
            
            # Step 3: Compute collision-free path to pre-insert position (25cm in front of hollow)
            self.get_logger().info(f'{arm}: Planning collision-free path to pre-insert position (25cm in front of hollow)...')
            
            # Try IK first (simpler, might work if path is clear)
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
                    raise RuntimeError('Cartesian path planning failed for pre-insert position')
            
            # Step 3.5: Apply 45-degree J7 offset to rotate gripper base straight toward hollow
            # This rotates the gripper (and held solid object) 45 degrees so its base faces straight to the hollow
            self.get_logger().info(f'{arm}: Applying 45-degree J7 offset to rotate gripper base straight toward hollow...')
            time.sleep(0.2)  # Brief wait to ensure previous motion is complete
            
            # Get current joint state
            if not self.joint_state:
                self.get_logger().warn(f'{arm}: No joint state available for J7 offset')
            else:
                try:
                    # Get current joint positions for this arm
                    current_joints = []
                    for joint_name in self.joint_names[arm]:
                        idx = self.joint_state.name.index(joint_name)
                        current_joints.append(self.joint_state.position[idx])
                    
                    # Apply 45-degree offset to J7 (index 6) - this rotates the gripper base
                    # J7 rotation will rotate the gripper and held object 45 degrees around the wrist axis
                    new_joints = list(current_joints)
                    new_joints[6] += (math.pi / 4.0)  # +45 degrees to rotate gripper base straight
                    self.send_joint_trajectory(new_joints, arm=arm, seconds=1.0)
                    self.get_logger().info(f'{arm}: Applied 45-degree J7 offset (J7: {current_joints[6]:.3f} -> {new_joints[6]:.3f}) - gripper base now faces straight toward hollow')
                    time.sleep(0.5)  # Brief wait after J7 offset to ensure rotation completes
                    
                    # CRITICAL: After J7 offset, get the NEW current pose to use as starting point for forward movement
                    # Wait a moment for joint state to update
                    time.sleep(0.3)
                    # Get updated joint state after J7 rotation
                    if self.joint_state:
                        updated_joints = []
                        for joint_name in self.joint_names[arm]:
                            idx = self.joint_state.name.index(joint_name)
                            updated_joints.append(self.joint_state.position[idx])
                        # Recompute the current pose from updated joints using IK/FK
                        # For now, we'll use the updated joints to compute IK to the same position but with new J7
                        # Actually, just update pre_insert_pose's orientation implicitly through the joint state
                except (ValueError, IndexError) as e:
                    self.get_logger().warn(f'{arm}: Failed to apply J7 offset: {e}')
            
            # Step 3.6: ON-THE-FLY ADJUSTMENT - Maintain 25cm distance and align Y, Z axes
            # Get live poses of both solid and hollow objects
            # Maintain 25cm distance along X-axis while aligning solid center Y,Z with hollow center Y,Z
            self.get_logger().info(f'{arm}: Maintaining 25cm distance and aligning solid center Y,Z with hollow center Y,Z using live poses...')
            time.sleep(0.2)  # Wait for pose updates
            
            # Re-read live poses of both objects
            solid_live_data = self.get_pose(solid_name)
            updated_hollow_data = self.get_pose(hollow_name)
            
            if not solid_live_data or not updated_hollow_data:
                self.get_logger().warn(f'{arm}: Could not get live poses (solid: {solid_live_data is not None}, hollow: {updated_hollow_data is not None}), skipping X-axis alignment')
                current_ee_data = self.get_ee_pose(arm)
            else:
                # Get current end-effector pose
                current_ee_data = self.get_ee_pose(arm)
                if current_ee_data:
                    # Get live positions
                    solid_center_x = solid_live_data['position'][0]
                    solid_center_y = solid_live_data['position'][1]
                    solid_center_z = solid_live_data['position'][2]
                    
                    hollow_center_x = updated_hollow_data['position'][0]
                    hollow_center_y = updated_hollow_data['position'][1]
                    hollow_center_z = updated_hollow_data['position'][2]
                    
                    # Current gripper TCP position
                    ee_x = current_ee_data[0]
                    ee_y = current_ee_data[1]
                    ee_z = current_ee_data[2]
                    
                    # OPTION 1: Calculate gripper-to-solid offset (accounts for solid not being perfectly centered in gripper)
                    gripper_to_solid_y_offset = solid_center_y - ee_y
                    gripper_to_solid_z_offset = solid_center_z - ee_z
                    self.get_logger().info(f'{arm}: Gripper-to-solid offset: Y={gripper_to_solid_y_offset:.4f}m, Z={gripper_to_solid_z_offset:.4f}m')
                    
                    # MAINTAIN 25cm distance but align Y and Z axes
                    # Keep gripper X at 25cm in front of hollow: target_ee_x = hollow_center_x - 0.25
                    # But align solid center Y and Z with hollow center Y and Z
                    
                    # Target gripper X: Maintain 25cm distance from hollow center
                    target_ee_x = hollow_center_x - 0.25  # 25cm in front of hollow center
                    
                    # OPTION 1: To align solid center Y with hollow center Y (accounting for offset):
                    # Current: solid_center_y = ee_y + gripper_to_solid_y_offset
                    # We want: solid_center_y = hollow_center_y
                    # So: target_ee_y + gripper_to_solid_y_offset = hollow_center_y
                    # Therefore: target_ee_y = hollow_center_y - gripper_to_solid_y_offset
                    target_ee_y = hollow_center_y - gripper_to_solid_y_offset
                    
                    # OPTION 1: To align solid center Z with hollow center Z (accounting for offset):
                    # Current: solid_center_z = ee_z + gripper_to_solid_z_offset
                    # We want: solid_center_z = hollow_center_z
                    # So: target_ee_z + gripper_to_solid_z_offset = hollow_center_z
                    # Therefore: target_ee_z = hollow_center_z - gripper_to_solid_z_offset
                    target_ee_z = hollow_center_z - gripper_to_solid_z_offset
                    
                    # Calculate offsets from current gripper position
                    total_x_offset = target_ee_x - ee_x
                    total_y_offset = target_ee_y - ee_y
                    total_z_offset = target_ee_z - ee_z
                    
                    # If offset is significant (>2mm), adjust gripper position
                    max_offset = max(abs(total_x_offset), abs(total_y_offset), abs(total_z_offset))
                    if max_offset > 0.002:  # 2mm threshold
                        self.get_logger().info(f'{arm}: Aligning solid center (Y={solid_center_y:.4f}, Z={solid_center_z:.4f}) with hollow center (Y={hollow_center_y:.4f}, Z={hollow_center_z:.4f})')
                        self.get_logger().info(f'{arm}: Maintaining 25cm distance: gripper X={target_ee_x:.4f} (hollow X={hollow_center_x:.4f} - 0.25m)')
                        self.get_logger().info(f'{arm}: Required offsets: X={total_x_offset:.4f}m (maintain 25cm), Y={total_y_offset:.4f}m (align centers), Z={total_z_offset:.4f}m (align centers)')
                        
                        # Create adjustment pose: maintain 25cm distance, align Y and Z
                        adjusted_ee_world = Pose()
                        adjusted_ee_world.position.x = target_ee_x  # Maintain 25cm in front
                        adjusted_ee_world.position.y = target_ee_y  # Align solid center Y with hollow center Y
                        adjusted_ee_world.position.z = target_ee_z  # Align solid center Z with hollow center Z
                        # Keep current orientation (after J7 rotation)
                        qx, qy, qz, qw = _euler_to_quaternion(current_ee_data[3], current_ee_data[4], current_ee_data[5])
                        adjusted_ee_world.orientation.x = qx
                        adjusted_ee_world.orientation.y = qy
                        adjusted_ee_world.orientation.z = qz
                        adjusted_ee_world.orientation.w = qw
                        
                        # Transform to robot frame
                        adjusted_ee_robot = self._transform_to_robot_frame(adjusted_ee_world, arm)
                        adjusted_ee_robot.orientation = adjusted_ee_world.orientation
                        
                        # Get current pose in robot frame for waypoint
                        current_ee_pose_world = Pose()
                        current_ee_pose_world.position.x = ee_x
                        current_ee_pose_world.position.y = ee_y
                        current_ee_pose_world.position.z = ee_z
                        current_ee_pose_world.orientation.x = qx
                        current_ee_pose_world.orientation.y = qy
                        current_ee_pose_world.orientation.z = qz
                        current_ee_pose_world.orientation.w = qw
                        current_ee_pose_robot = self._transform_to_robot_frame(current_ee_pose_world, arm)
                        current_ee_pose_robot.orientation = current_ee_pose_world.orientation
                        
                        # Adjustment movement to maintain 25cm distance and align Y,Z axes
                        adjustment_waypoints = [current_ee_pose_robot, adjusted_ee_robot]
                        adjustment_joints = self._compute_cartesian_path(adjustment_waypoints, arm=arm)
                        
                        if adjustment_joints:
                            self.send_joint_trajectory(adjustment_joints, arm=arm, seconds=2.0)
                            self.get_logger().info(f'{arm}: Successfully maintained 25cm distance and aligned solid center Y,Z with hollow center Y,Z')
                            time.sleep(0.3)  # Wait for adjustment to complete
                            
                            # OPTION 2 & 4: Re-read poses after alignment and verify/correct alignment
                            self.get_logger().info(f'{arm}: Re-reading object poses after alignment to verify...')
                            time.sleep(0.2)  # Wait for poses to update
                            
                            # Re-read live poses to verify alignment
                            verify_solid_data = self.get_pose(solid_name)
                            verify_hollow_data = self.get_pose(hollow_name)
                            
                            if verify_solid_data and verify_hollow_data:
                                verify_solid_y = verify_solid_data['position'][1]
                                verify_solid_z = verify_solid_data['position'][2]
                                verify_hollow_y = verify_hollow_data['position'][1]
                                verify_hollow_z = verify_hollow_data['position'][2]
                                
                                # Calculate remaining misalignment
                                y_misalignment = verify_hollow_y - verify_solid_y
                                z_misalignment = verify_hollow_z - verify_solid_z
                                max_misalignment = max(abs(y_misalignment), abs(z_misalignment))
                                
                                self.get_logger().info(f'{arm}: Post-alignment verification: solid Y={verify_solid_y:.4f}, hollow Y={verify_hollow_y:.4f}, diff={y_misalignment:.4f}m')
                                self.get_logger().info(f'{arm}: Post-alignment verification: solid Z={verify_solid_z:.4f}, hollow Z={verify_hollow_z:.4f}, diff={z_misalignment:.4f}m')
                                
                                # OPTION 4: If still misaligned (>1mm), apply correction
                                if max_misalignment > 0.001:  # 1mm threshold for refinement
                                    self.get_logger().info(f'{arm}: Still misaligned (Y diff: {y_misalignment:.4f}m, Z diff: {z_misalignment:.4f}m), applying correction...')
                                    
                                    # Get current EE pose after first alignment
                                    verify_ee_data = self.get_ee_pose(arm)
                                    if verify_ee_data:
                                        verify_ee_y = verify_ee_data[1]
                                        verify_ee_z = verify_ee_data[2]
                                        
                                        # Calculate correction needed (accounting for gripper-to-solid offset)
                                        # We want: solid_center_y = hollow_center_y
                                        # Current: solid_center_y = verify_ee_y + gripper_to_solid_y_offset
                                        # Required: target_ee_y + gripper_to_solid_y_offset = hollow_center_y
                                        corrected_ee_y = verify_hollow_y - gripper_to_solid_y_offset
                                        corrected_ee_z = verify_hollow_z - gripper_to_solid_z_offset
                                        
                                        # Create correction pose (keep X and orientation the same)
                                        corrected_ee_world = Pose()
                                        corrected_ee_world.position.x = verify_ee_data[0]  # Keep X as is
                                        corrected_ee_world.position.y = corrected_ee_y
                                        corrected_ee_world.position.z = corrected_ee_z
                                        corrected_ee_world.orientation.x = qx
                                        corrected_ee_world.orientation.y = qy
                                        corrected_ee_world.orientation.z = qz
                                        corrected_ee_world.orientation.w = qw
                                        
                                        # Transform to robot frame
                                        corrected_ee_robot = self._transform_to_robot_frame(corrected_ee_world, arm)
                                        corrected_ee_robot.orientation = corrected_ee_world.orientation
                                        
                                        # Small correction movement
                                        current_verify_pose_world = Pose()
                                        current_verify_pose_world.position.x = verify_ee_data[0]
                                        current_verify_pose_world.position.y = verify_ee_y
                                        current_verify_pose_world.position.z = verify_ee_z
                                        current_verify_pose_world.orientation.x = qx
                                        current_verify_pose_world.orientation.y = qy
                                        current_verify_pose_world.orientation.z = qz
                                        current_verify_pose_world.orientation.w = qw
                                        current_verify_pose_robot = self._transform_to_robot_frame(current_verify_pose_world, arm)
                                        current_verify_pose_robot.orientation = current_verify_pose_world.orientation
                                        
                                        correction_waypoints = [current_verify_pose_robot, corrected_ee_robot]
                                        correction_joints = self._compute_cartesian_path(correction_waypoints, arm=arm)
                                        
                                        if correction_joints:
                                            self.send_joint_trajectory(correction_joints, arm=arm, seconds=1.0)
                                            self.get_logger().info(f'{arm}: Applied alignment correction (Y: {verify_ee_y:.4f} -> {corrected_ee_y:.4f}, Z: {verify_ee_z:.4f} -> {corrected_ee_z:.4f})')
                                            time.sleep(0.2)
                                            
                                            # Update current_ee_data after correction
                                            current_ee_data = self.get_ee_pose(arm)
                                        else:
                                            self.get_logger().warn(f'{arm}: Failed to compute correction path, proceeding with current alignment')
                                else:
                                    self.get_logger().info(f'{arm}: ✓ Alignment verified: solid and hollow centers are aligned (within 1mm tolerance)')
                            
                            # Re-read EE pose after alignment (or correction)
                            current_ee_data = self.get_ee_pose(arm)
                        else:
                            self.get_logger().warn(f'{arm}: Failed to compute alignment path, proceeding with current position')
                    else:
                        self.get_logger().info(f'{arm}: Already aligned (X offset: {total_x_offset:.4f}m, Y offset: {total_y_offset:.4f}m, Z offset: {total_z_offset:.4f}m, all within tolerance)')
                else:
                    self.get_logger().warn(f'{arm}: Could not get current EE pose for alignment')
            
            # Step 3.7: FINAL ALIGNMENT VERIFICATION - Ensure perfect alignment before forward movement
            # Re-read poses one final time and verify/correct alignment to ensure both centers are perfectly aligned
            self.get_logger().info(f'{arm}: Final alignment verification before forward movement...')
            time.sleep(0.2)  # Wait for pose updates
            
            # Re-read live poses one final time
            final_solid_data = self.get_pose(solid_name)
            final_hollow_data = self.get_pose(hollow_name)
            
            if final_solid_data and final_hollow_data:
                final_solid_y = final_solid_data['position'][1]
                final_solid_z = final_solid_data['position'][2]
                final_hollow_y = final_hollow_data['position'][1]
                final_hollow_z = final_hollow_data['position'][2]
                
                # Get current EE pose
                final_ee_data = self.get_ee_pose(arm)
                if final_ee_data:
                    final_ee_y = final_ee_data[1]
                    final_ee_z = final_ee_data[2]
                    
                    # Recalculate gripper-to-solid offset (it might have changed slightly)
                    final_gripper_to_solid_y_offset = final_solid_y - final_ee_y
                    final_gripper_to_solid_z_offset = final_solid_z - final_ee_z
                    
                    # Calculate misalignment
                    y_misalignment = final_hollow_y - final_solid_y
                    z_misalignment = final_hollow_z - final_solid_z
                    max_misalignment = max(abs(y_misalignment), abs(z_misalignment))
                    
                    self.get_logger().info(f'{arm}: Final check - Solid Y={final_solid_y:.4f}, Hollow Y={final_hollow_y:.4f}, Diff={y_misalignment:.4f}m')
                    self.get_logger().info(f'{arm}: Final check - Solid Z={final_solid_z:.4f}, Hollow Z={final_hollow_z:.4f}, Diff={z_misalignment:.4f}m')
                    
                    # If misaligned (>0.5mm), apply final correction
                    if max_misalignment > 0.0005:  # 0.5mm threshold for final precision
                        self.get_logger().info(f'{arm}: Final correction needed (Y diff: {y_misalignment:.4f}m, Z diff: {z_misalignment:.4f}m), aligning centers perfectly...')
                        
                        # Calculate target gripper position to perfectly align centers
                        # We want: solid_center = hollow_center
                        # Since: solid_center = ee_position + gripper_to_solid_offset
                        # Therefore: target_ee_y + gripper_to_solid_y_offset = hollow_center_y
                        # So: target_ee_y = hollow_center_y - gripper_to_solid_y_offset
                        final_target_ee_y = final_hollow_y - final_gripper_to_solid_y_offset
                        final_target_ee_z = final_hollow_z - final_gripper_to_solid_z_offset
                        
                        # Create final correction pose (keep X and orientation)
                        final_corrected_world = Pose()
                        final_corrected_world.position.x = final_ee_data[0]  # Keep X (maintain 25cm distance)
                        final_corrected_world.position.y = final_target_ee_y  # Perfect Y alignment
                        final_corrected_world.position.z = final_target_ee_z  # Perfect Z alignment
                        # Keep current orientation (after J7 rotation)
                        qx, qy, qz, qw = _euler_to_quaternion(final_ee_data[3], final_ee_data[4], final_ee_data[5])
                        final_corrected_world.orientation.x = qx
                        final_corrected_world.orientation.y = qy
                        final_corrected_world.orientation.z = qz
                        final_corrected_world.orientation.w = qw
                        
                        # Transform to robot frame
                        final_corrected_robot = self._transform_to_robot_frame(final_corrected_world, arm)
                        final_corrected_robot.orientation = final_corrected_world.orientation
                        
                        # Current pose in robot frame
                        final_current_world = Pose()
                        final_current_world.position.x = final_ee_data[0]
                        final_current_world.position.y = final_ee_y
                        final_current_world.position.z = final_ee_z
                        final_current_world.orientation.x = qx
                        final_current_world.orientation.y = qy
                        final_current_world.orientation.z = qz
                        final_current_world.orientation.w = qw
                        final_current_robot = self._transform_to_robot_frame(final_current_world, arm)
                        final_current_robot.orientation = final_current_world.orientation
                        
                        # Final correction movement
                        final_correction_waypoints = [final_current_robot, final_corrected_robot]
                        final_correction_joints = self._compute_cartesian_path(final_correction_waypoints, arm=arm)
                        
                        if final_correction_joints:
                            self.send_joint_trajectory(final_correction_joints, arm=arm, seconds=1.5)
                            self.get_logger().info(f'{arm}: ✓ Final alignment correction applied - solid and hollow centers now perfectly aligned')
                            time.sleep(0.2)
                            
                            # Update current_ee_data after final correction
                            current_ee_data = self.get_ee_pose(arm)
                        else:
                            self.get_logger().warn(f'{arm}: Failed to compute final correction path, proceeding with current alignment')
                    else:
                        self.get_logger().info(f'{arm}: ✓ Perfect alignment confirmed! Solid and hollow centers are aligned (within 0.5mm tolerance)')
                        # Update current_ee_data (use existing)
                        current_ee_data = final_ee_data
                else:
                    self.get_logger().warn(f'{arm}: Could not get EE pose for final verification')
            else:
                self.get_logger().warn(f'{arm}: Could not get object poses for final verification')
            
            # Step 4: Move forward along X-axis (toward hollow)
            # CRITICAL: After J7 offset and final alignment verification, get the current pose for forward movement
            self.get_logger().info(f'{arm}: Moving forward 8cm along X-axis from current position (after final alignment verification)...')
            
            # Get current end-effector pose after all adjustments
            if not current_ee_data:
                self.get_logger().warn(f'{arm}: Cannot get current EE pose, using pre_insert_pose as fallback')
                current_ee_pose_robot = pre_insert_pose
                current_ee_pose_world = None
            else:
                # Convert current EE pose to robot frame for use as starting point
                current_ee_pose_world = Pose()
                current_ee_pose_world.position.x = current_ee_data[0]
                current_ee_pose_world.position.y = current_ee_data[1]
                current_ee_pose_world.position.z = current_ee_data[2]
                # Use current orientation (from FK) - this includes the J7 rotation
                qx, qy, qz, qw = _euler_to_quaternion(current_ee_data[3], current_ee_data[4], current_ee_data[5])
                current_ee_pose_world.orientation.x = qx
                current_ee_pose_world.orientation.y = qy
                current_ee_pose_world.orientation.z = qz
                current_ee_pose_world.orientation.w = qw
                
                # Transform to robot frame
                current_ee_pose_robot = self._transform_to_robot_frame(current_ee_pose_world, arm)
                current_ee_pose_robot.orientation = current_ee_pose_world.orientation
            
            # Calculate insert pose: 2cm forward from CURRENT position (after adjustment)
            # CRITICAL: Use current position (after J7 offset and alignment) as starting point
            if not current_ee_data or not current_ee_pose_world:
                self.get_logger().warn(f'{arm}: Cannot calculate insert pose from current position, using original calculation')
                insert_pose_robot = self._transform_to_robot_frame(world_insert_pose, arm)
                insert_pose_robot.orientation = world_insert_pose.orientation
            else:
                # Calculate insert pose: current position + 8cm forward along X-axis (toward hollow)
                # CRITICAL: Keep Y and Z exactly the same to maintain perfect alignment during forward movement
                # This ensures the solid center moves straight into the hollow center
                insert_pose_world = Pose()
                insert_pose_world.position.x = current_ee_pose_world.position.x + 0.08  # 8cm forward (0.08m)
                insert_pose_world.position.y = current_ee_pose_world.position.y  # Keep Y exactly the same (maintains Y alignment)
                insert_pose_world.position.z = current_ee_pose_world.position.z  # Keep Z exactly the same (maintains Z alignment)
                # Use current orientation (after J7 rotation)
                insert_pose_world.orientation = current_ee_pose_world.orientation
                
                self.get_logger().info(f'{arm}: Insert pose calculated: 8cm forward from current position (X={insert_pose_world.position.x:.3f}m)')
                self.get_logger().info(f'{arm}: Y and Z maintained at aligned values (Y={insert_pose_world.position.y:.4f}m, Z={insert_pose_world.position.z:.4f}m)')
                
                # Transform to robot frame
                insert_pose_robot = self._transform_to_robot_frame(insert_pose_world, arm)
                insert_pose_robot.orientation = insert_pose_world.orientation
            
            # Use Cartesian path for precise 8cm forward movement from CURRENT position
            # CRITICAL: This maintains perfect Y,Z alignment throughout forward movement (straight insertion)
            # Y and Z stay exactly the same, only X moves forward - ensures solid goes straight into hole
            waypoints_insert = [current_ee_pose_robot, insert_pose_robot]
            joints_insert = self._compute_cartesian_path(waypoints_insert, arm=arm)
            
            if joints_insert:
                self.send_joint_trajectory(joints_insert, arm=arm, seconds=3.0)
                self.get_logger().info(f'{arm}: Successfully moved forward 8cm with perfect Y,Z alignment maintained!')
            else:
                # Fallback: Try IK
                self.get_logger().warn(f'{arm}: Cartesian path for 8cm forward failed, trying IK...')
                try:
                    insert_joints_ik = self.compute_ik(insert_pose_robot, arm=arm)
                    self.send_joint_trajectory(insert_joints_ik, arm=arm, seconds=3.0)
                    self.get_logger().info(f'{arm}: Successfully moved forward 8cm via IK with perfect alignment!')
                except Exception as ik_err2:
                    self.get_logger().error(f'{arm}: Failed to move forward 8cm: {ik_err2}')
                    return False
            
            self.get_logger().info(f'{arm}: Insert sequence complete! Reached pre-insert (25cm front), applied J7 offset, perfectly aligned centers in Y,Z, then moved forward 8cm straight into hole.')
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
        
        # Get the EE pose of the arm holding the object
        ee_data = self.get_ee_pose(arm)
        if not ee_data:
            self.get_logger().warn(f'Cannot get EE pose for {arm} to add held object collision')
            return
        
        ee_x, ee_y, ee_z, ee_roll, ee_pitch, ee_yaw = ee_data
        
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
        for i in range(3):
            self.planning_scene_pub.publish(scene)
            time.sleep(0.05)
        self.get_logger().info(f'Updated {object_name} held by {arm} as collision object at ({ee_x:.3f}, {ee_y:.3f}, {ee_z:.3f})')
    
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
        
        # Objects only
        objects = ['red_block', 'green_block', 'red_solid', 'green_solid', 'red_hollow', 'green_hollow']
        
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


def main():
    """Main entry point."""
    gui = DualPandaUnifiedGUI()
    try:
        gui.run()
    except KeyboardInterrupt:
        gui.shutdown()


if __name__ == '__main__':
    main()

