#!/usr/bin/env python3

"""Tkinter GUI to command dual Panda end-effector poses via MoveIt's IK service."""

import math
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from typing import List, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DualPandaIKNode(Node):
    """ROS2 node handling IK requests and trajectory execution for both pandas."""

    def __init__(self) -> None:
        super().__init__('dual_panda_ik_gui_node')
        
        # Robot world poses (from launch file)
        self.robot_poses = {
            'panda1': {'x': 0.0, 'y': 0.15, 'z': 0.0, 'yaw': 0.0},
            'panda2': {'x': 1.4, 'y': -0.15, 'z': 0.0, 'yaw': math.pi}
        }
        
        self.joint_names: List[str] = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7',
        ]
        # Gripper length offset (wrist to fingertip center)
        # When grasping top-down, we must add this to the target Z
        self.gripper_length = 0.11  # 11cm
        self.neutral_pose: List[float] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]

        # Panda 1 publishers and clients
        self.joint_pub_panda1 = self.create_publisher(
            JointTrajectory,
            '/panda1/panda_arm_controller/joint_trajectory',
            10,
        )
        self.ik_client_panda1 = self.create_client(GetPositionIK, '/panda1/compute_ik')
        self.gripper_client_panda1 = ActionClient(
            self,
            GripperCommand,
            '/panda1/panda_gripper_controller/gripper_cmd',
        )

        # Panda 2 publishers and clients
        self.joint_pub_panda2 = self.create_publisher(
            JointTrajectory,
            '/panda2/panda_arm_controller/joint_trajectory',
            10,
        )
        self.ik_client_panda2 = self.create_client(GetPositionIK, '/panda2/compute_ik')
        self.gripper_client_panda2 = ActionClient(
            self,
            GripperCommand,
            '/panda2/panda_gripper_controller/gripper_cmd',
        )

        # Wait for services
        self.get_logger().info('Waiting for MoveIt compute_ik services...')
        if not self.ik_client_panda1.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Panda1 compute_ik service not available.')
        if not self.ik_client_panda2.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Panda2 compute_ik service not available.')

        self.get_logger().info('Waiting for gripper action servers...')
        self.gripper_client_panda1.wait_for_server(timeout_sec=5.0)
        self.gripper_client_panda2.wait_for_server(timeout_sec=5.0)
        
        # Planning scene publishers
        self.planning_scene_pub_panda1 = self.create_publisher(
            PlanningScene,
            '/panda1/monitored_planning_scene',
            10
        )
        self.planning_scene_pub_panda2 = self.create_publisher(
            PlanningScene,
            '/panda2/monitored_planning_scene',
            10
        )
        
        # Cartesian path clients
        self.cartesian_path_client_panda1 = self.create_client(
            GetCartesianPath,
            '/panda1/compute_cartesian_path'
        )
        self.cartesian_path_client_panda2 = self.create_client(
            GetCartesianPath,
            '/panda2/compute_cartesian_path'
        )
        
        # Wait for services
        self.get_logger().info('Waiting for MoveIt services...')
        if not self.cartesian_path_client_panda1.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Panda1 cartesian path service not available.')
        if not self.cartesian_path_client_panda2.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Panda2 cartesian path service not available.')
        
        # Table and object definitions
        # Table: 1.0m x 3.0m x 0.2m at x=0.7, y=0, z=0.1 (workbench from world file)
        # Workbench pose (0.7, 0, 0.1) with size (1.0, 3.0, 0.2) means:
        # - Center is at z=0.1, top is at z=0.2
        self.table_pose = Pose()
        self.table_pose.position.x = 0.7
        self.table_pose.position.y = 0.0
        self.table_pose.position.z = 0.1  # Center of table (half height = 0.1)
        self.table_pose.orientation.w = 1.0
        self.table_size = [1.0, 3.0, 0.2]  # Width (x), depth (y), height (z) from world file
        self.table_height = 0.2  # Top of table is at z=0.2
        
        # Object definitions (from world file with updated positions)
        # POSITIONS are CENTERS, not tops (SDF convention)
        self.objects = {
            'red_block': {
                'position': [0.4, -0.02, 0.225],  # x, y, z (center)
                'size': [0.05, 0.05, 0.05],  # dimensions
                'yaw': 0.0
            },
            'green_block': {
                'position': [0.45, 0.2, 0.225],
                'size': [0.05, 0.05, 0.05],
                'yaw': 0.0
            },
            'red_solid': {
                'position': [0.6, -0.2, 0.25],  # 80mm x 30mm x 30mm
                'size': [0.08, 0.03, 0.03],
                'yaw': 0.0
            },
            'green_solid': {
                'position': [0.6, 0.2, 0.25],
                'size': [0.08, 0.03, 0.03],
                'yaw': 0.0
            },
            'red_hollow': {
                'position': [0.9, -0.2, 0.231],  # Receptacle (static)
                'size': [0.08, 0.05, 0.05],
                'yaw': -1.5708  # -90 degrees
            },
            'green_hollow': {
                'position': [0.9, 0.2, 0.231],
                'size': [0.08, 0.05, 0.05],
                'yaw': -1.5708
            }
        }
        
        # Lock for thread-safe access to objects dictionary
        self.objects_lock = threading.Lock()
        
        # Initialize planning scene
        self._setup_planning_scene()
        
        # Subscribe to Ground Truth poses from Gazebo Bridge
        self.pose_subs = []
        for obj_name in self.objects.keys():
            topic = f'/model/{obj_name}/pose'
            self.get_logger().info(f'Subscribing to ground truth: {topic}')
            sub = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, name=obj_name: self._pose_callback(msg, name),
                10
            )
            self.pose_subs.append(sub)
            
        self.get_logger().info('Dual Panda IK GUI Node initialized. Receiving ground truth poses...')

    def _pose_callback(self, msg: PoseStamped, obj_name: str):
        """Update object pose from ground truth."""
        with self.objects_lock:
            if obj_name in self.objects:
                # Update position
                self.objects[obj_name]['position'] = [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                ]
                
                # Update Yaw from Quaternion
                q = [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w
                ]
                # Manual yaw calculation to avoid numpy issues
                siny_cosp = 2.0 * (q[3] * q[2] + q[0] * q[1])
                cosy_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
                yaw = math.atan2(siny_cosp, cosy_cosp)
                
                self.objects[obj_name]['yaw'] = yaw

    def _transform_to_robot_frame(self, world_pose: Pose, arm: str) -> Pose:
        """Transform a pose from world frame to robot base frame."""
        robot = self.robot_poses[arm]
        
        # Translate
        dx = world_pose.position.x - robot['x']
        dy = world_pose.position.y - robot['y']
        dz = world_pose.position.z - robot['z']
        
        # Rotate (inverse of robot yaw)
        # x' = x cos(-yaw) - y sin(-yaw)
        # y' = x sin(-yaw) + y cos(-yaw)
        # cos(-theta) = cos(theta), sin(-theta) = -sin(theta)
        cos_yaw = math.cos(-robot['yaw'])
        sin_yaw = math.sin(-robot['yaw'])
        
        local_x = dx * cos_yaw - dy * sin_yaw
        local_y = dx * sin_yaw + dy * cos_yaw
        local_z = dz
        
        local_pose = Pose()
        local_pose.position.x = local_x
        local_pose.position.y = local_y
        local_pose.position.z = local_z
        
        # Orientation requires converting quat to euler, adjusting yaw, back to quat
        # Simplified: assume objects are flat and we only care about yaw
        # For now, we construct orientation in the caller using _euler_to_quaternion
        # This function just transforms position
        local_pose.orientation = world_pose.orientation
        
        return local_pose

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------
    def compute_ik(self, pose: Pose, arm: str = 'panda1', base_frame: str = 'panda_link0', move_group: str = 'panda_arm') -> List[float]:
        """Compute IK for specified arm."""
        ik_client = self.ik_client_panda1 if arm == 'panda1' else self.ik_client_panda2
        
        request = GetPositionIK.Request()
        request.ik_request.group_name = move_group
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=2.0).to_msg()

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        request.ik_request.pose_stamped = pose_stamped

        future = ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if not response:
            raise RuntimeError('IK service call failed (no response).')
        if response.error_code.val != response.error_code.SUCCESS:
            raise RuntimeError(f'IK failed with error code {response.error_code.val}')

        joint_state = response.solution.joint_state
        joint_map = {name: position for name, position in zip(joint_state.name, joint_state.position)}

        try:
            return [joint_map[name] for name in self.joint_names]
        except KeyError as exc:
            raise RuntimeError(f'Missing joint in IK solution: {exc}') from exc

    def send_joint_trajectory(self, joint_positions: List[float], arm: str = 'panda1', seconds: float = 3.0) -> None:
        """Send joint trajectory to specified arm."""
        joint_pub = self.joint_pub_panda1 if arm == 'panda1' else self.joint_pub_panda2
        
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
        traj.points.append(point)

        joint_pub.publish(traj)

    def send_gripper_goal(self, width: float, arm: str = 'panda1') -> None:
        """Send gripper command to specified arm."""
        gripper_client = self.gripper_client_panda1 if arm == 'panda1' else self.gripper_client_panda2
        
        goal = GripperCommand.Goal()
        goal.command.position = width
        goal.command.max_effort = 20.0

        if not gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'Gripper action server unavailable for {arm}.')
            return

        future = gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error(f'Failed to send gripper goal for {arm} (no goal handle).')
            return
        if not goal_handle.accepted:
            self.get_logger().warn(f'Gripper goal rejected for {arm}.')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
    
    # ------------------------------------------------------------------
    # Planning Scene helpers
    # ------------------------------------------------------------------
    def _setup_planning_scene(self) -> None:
        """Set up planning scene with table and objects for both arms."""
        self.get_logger().info('Setting up planning scene...')
        
        with self.objects_lock:
            objects_copy = {k: v.copy() for k, v in self.objects.items()}
        
        # Add table and objects to both planning scenes
        for arm in ['panda1', 'panda2']:
            scene_pub = self.planning_scene_pub_panda1 if arm == 'panda1' else self.planning_scene_pub_panda2
            base_frame = 'panda_link0'  # Local frame for each robot
            
            # Create planning scene message
            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects = []
            scene.robot_state.is_diff = True
            
            # Add table (transform to robot frame)
            table_obj = CollisionObject()
            table_obj.header.frame_id = base_frame
            table_obj.header.stamp = self.get_clock().now().to_msg()
            table_obj.id = 'table'
            table_obj.operation = CollisionObject.ADD
            
            table_primitive = SolidPrimitive()
            table_primitive.type = SolidPrimitive.BOX
            table_primitive.dimensions = self.table_size
            
            # Transform table pose
            table_local = self._transform_to_robot_frame(self.table_pose, arm)
            # Correct orientation for Panda 2 (table yaw should be rotated)
            # Table is symmetric, but if it wasn't, we'd need to rotate dimensions or orientation
            # For box, rotating 180 around Z is same shape
            
            table_obj.primitives = [table_primitive]
            table_obj.primitive_poses = [table_local]
            scene.world.collision_objects.append(table_obj)
            
            # Add all objects (transform to robot frame)
            for obj_name, obj_data in objects_copy.items():
                obj = CollisionObject()
                obj.header.frame_id = base_frame
                obj.id = obj_name
                obj.operation = CollisionObject.ADD
                
                obj_primitive = SolidPrimitive()
                obj_primitive.type = SolidPrimitive.BOX
                obj_primitive.dimensions = obj_data['size']
                
                # Create world pose
                world_pose = Pose()
                world_pose.position.x = obj_data['position'][0]
                world_pose.position.y = obj_data['position'][1]
                world_pose.position.z = obj_data['position'][2]  # Center
                
                # Transform to local
                local_pose = self._transform_to_robot_frame(world_pose, arm)
                
                # Calculate local yaw
                # Yaw_local = Yaw_world - Robot_Yaw
                obj_yaw_local = obj_data['yaw'] - self.robot_poses[arm]['yaw']
                
                qx, qy, qz, qw = _euler_to_quaternion(0.0, 0.0, obj_yaw_local)
                local_pose.orientation.x = qx
                local_pose.orientation.y = qy
                local_pose.orientation.z = qz
                local_pose.orientation.w = qw
                
                obj.primitives = [obj_primitive]
                obj.primitive_poses = [local_pose]
                scene.world.collision_objects.append(obj)
            
            # Publish scene
            scene_pub.publish(scene)
            self.get_logger().info(f'Planning scene published for {arm}')
        
        # Wait a bit for scene to be processed
        import time
        time.sleep(1.0)
    
    def _compute_cartesian_path(self, waypoints: List[Pose], arm: str = 'panda1') -> List[float]:
        """Compute cartesian path through waypoints."""
        cartesian_client = self.cartesian_path_client_panda1 if arm == 'panda1' else self.cartesian_path_client_panda2
        
        request = GetCartesianPath.Request()
        # Use "panda_link0" as the planning frame - MoveIt works in robot frame
        request.header.frame_id = 'panda_link0'
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = 'panda_arm'
        request.waypoints = waypoints
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        
        future = cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        
        if not response:
            raise RuntimeError('Cartesian path service call failed.')
        
        if response.fraction < 1.0:
            raise RuntimeError(f'Cartesian path incomplete: {response.fraction * 100:.1f}% feasible')
        
        # Extract joint positions from trajectory
        if not response.solution.joint_trajectory.points:
            raise RuntimeError('No trajectory points in cartesian path response')
        
        # Return joint positions from last waypoint
        last_point = response.solution.joint_trajectory.points[-1]
        joint_map = {name: pos for name, pos in zip(
            response.solution.joint_trajectory.joint_names,
            last_point.positions
        )}
        
        return [joint_map[name] for name in self.joint_names]
    
    def pick_object(self, object_name: str, arm: str = 'panda1') -> bool:
        """
        Pick an object using 5-step pipeline:
        1. Open gripper FIRST
        2. Pre-grasp pose (15cm above object)
        3. Linear descent to object center
        4. Grasp (close gripper, attach object)
        5. Linear retreat (lift 15cm up)
        """
        with self.objects_lock:
            if object_name not in self.objects:
                self.get_logger().error(f'Unknown object: {object_name}')
                return False
            
            obj_data = self.objects[object_name].copy()
        
        # Use raw center position from SDF
        obj_world_pos = obj_data['position']
        obj_size = obj_data['size']
        obj_yaw = obj_data['yaw']
        
        # Calculate object center and top (World Frame)
        # Use Ground Truth Z directly from perception
        obj_center_z_world = obj_world_pos[2]
        obj_top_z_world = obj_center_z_world + obj_size[2] / 2
        
        try:
            import time
            
            # Step 1: Open gripper FIRST
            self.get_logger().info(f'{arm}: Opening gripper...')
            self.send_gripper_goal(0.04, arm=arm)  # Open gripper
            time.sleep(1.5)  # Wait for gripper to open
            
            # Step 2: Pre-grasp pose (15cm above object top)
            # Construct World Pose
            world_pre_grasp = Pose()
            world_pre_grasp.position.x = obj_world_pos[0]
            world_pre_grasp.position.y = obj_world_pos[1]
            world_pre_grasp.position.z = obj_top_z_world + 0.15
            
            # Transform to Robot Frame
            pre_grasp_pose = self._transform_to_robot_frame(world_pre_grasp, arm)
            
            # Orientation: Look down (Roll=180)
            # Need to adjust yaw relative to robot
            # Target World Yaw = obj_yaw
            # Target Local Yaw = obj_yaw - robot_yaw
            # ADD 45 DEGREE OFFSET to align fingers with faces (Panda gripper fingers are diagonal in default frame?)
            target_yaw_local = obj_yaw - self.robot_poses[arm]['yaw'] + math.pi / 4
            
            qx, qy, qz, qw = _euler_to_quaternion(math.pi, 0.0, target_yaw_local)
            pre_grasp_pose.orientation.x = qx
            pre_grasp_pose.orientation.y = qy
            pre_grasp_pose.orientation.z = qz
            pre_grasp_pose.orientation.w = qw
            
            # Move to pre-grasp pose using IK
            self.get_logger().info(f'{arm}: Moving to pre-grasp pose (15cm above object)...')
            # Ensure we use panda_link0 as base frame
            pre_grasp_joints = self.compute_ik(pre_grasp_pose, arm=arm, base_frame='panda_link0')
            self.send_joint_trajectory(pre_grasp_joints, arm=arm, seconds=3.0)
            time.sleep(3.5)
            
            # Step 3: Linear descent to object center (cartesian path)
            grasp_pose = Pose()
            grasp_pose.position = pre_grasp_pose.position
            # We want the gripper FINGERTIPS to be at object center
            # So TCP (wrist) should be at Object Center + Gripper Length
            # Transform object center to local frame
            world_center_pose = Pose()
            world_center_pose.position.x = obj_world_pos[0]
            world_center_pose.position.y = obj_world_pos[1]
            world_center_pose.position.z = obj_center_z_world
            
            local_center = self._transform_to_robot_frame(world_center_pose, arm)
            
            # TCP Target = Center + Gripper Length
            grasp_pose.position.z = local_center.position.z + self.gripper_length
            grasp_pose.orientation = pre_grasp_pose.orientation
            
            self.get_logger().info(f'{arm}: Linear descent to grasp pose (TCP z={grasp_pose.position.z:.3f})...')
            waypoints = [pre_grasp_pose, grasp_pose]
            grasp_joints = self._compute_cartesian_path(waypoints, arm=arm)
            self.send_joint_trajectory(grasp_joints, arm=arm, seconds=2.0)
            time.sleep(2.5)
            
            # Step 4: Grasp (close gripper and attach object)
            self.get_logger().info(f'{arm}: Closing gripper to grasp object...')
            self.send_gripper_goal(0.01, arm=arm)  # Close gripper
            time.sleep(1.5)  # Wait for gripper to close
            
            # Attach object to gripper (planning scene)
            scene_pub = self.planning_scene_pub_panda1 if arm == 'panda1' else self.planning_scene_pub_panda2
            # base_frame = 'panda1_link0' if arm == 'panda1' else 'panda2_link0'
            # No need for base_frame here, just link names
            eef_link = 'panda1_hand' if arm == 'panda1' else 'panda2_hand'
            
            scene = PlanningScene()
            scene.is_diff = True
            
            attached_obj = AttachedCollisionObject()
            attached_obj.object.id = object_name
            attached_obj.object.operation = CollisionObject.REMOVE
            attached_obj.link_name = eef_link
            attached_obj.touch_links = [eef_link, f'{arm}_hand', f'{arm}_leftfinger', f'{arm}_rightfinger']
            
            scene.robot_state.attached_collision_objects = [attached_obj]
            scene_pub.publish(scene)
            time.sleep(0.5)
            
            # Step 5: Linear retreat (lift 15cm up)
            lift_pose = Pose()
            lift_pose.position = grasp_pose.position
            lift_pose.position.z += 0.15  # Lift 15cm relative to grasp (local z is up)
            lift_pose.orientation = grasp_pose.orientation
            
            self.get_logger().info(f'{arm}: Lifting object 15cm above table...')
            lift_waypoints = [grasp_pose, lift_pose]
            lift_joints = self._compute_cartesian_path(lift_waypoints, arm=arm)
            self.send_joint_trajectory(lift_joints, arm=arm, seconds=2.0)
            time.sleep(2.5)
            
            self.get_logger().info(f'{arm}: Pick complete!')
            return True
            
        except Exception as e:
            self.get_logger().error(f'{arm}: Pick failed: {e}')
            return False


class DualPandaIKGUI:
    """Tkinter front-end for controlling both Panda arms."""

    def __init__(self) -> None:
        # Initialize rclpy if not already initialized
        try:
            rclpy.init()
        except RuntimeError:
            # Already initialized, that's fine
            pass
        self.node = DualPandaIKNode()

        self.root = tk.Tk()
        self.root.title('Dual Panda End-Effector IK Control')
        self.root.protocol('WM_DELETE_WINDOW', self.shutdown)

        self.status_var_panda1 = tk.StringVar(value='Ready')
        self.status_var_panda2 = tk.StringVar(value='Ready')
        self._build_layout()

        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.spin_thread.start()

    # ------------------------------------------------------------------
    def _build_layout(self) -> None:
        main_frame = ttk.Frame(self.root, padding=12)
        main_frame.grid(column=0, row=0, sticky='nsew')
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # Title
        title_label = ttk.Label(main_frame, text='Dual Panda Arm Control', font=('Arial', 16, 'bold'))
        title_label.grid(column=0, row=0, columnspan=2, pady=(0, 20))

        # Create frames for each arm
        panda1_frame = ttk.LabelFrame(main_frame, text='Panda 1', padding=10)
        panda1_frame.grid(column=0, row=1, padx=10, pady=5, sticky='nsew')
        
        panda2_frame = ttk.LabelFrame(main_frame, text='Panda 2', padding=10)
        panda2_frame.grid(column=1, row=1, padx=10, pady=5, sticky='nsew')

        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)

        # Build Panda 1 controls
        self._build_arm_controls(panda1_frame, 'panda1')
        
        # Build Panda 2 controls
        self._build_arm_controls(panda2_frame, 'panda2')

        # Global controls
        global_frame = ttk.LabelFrame(main_frame, text='Global Controls', padding=10)
        global_frame.grid(column=0, row=2, columnspan=2, pady=10, sticky='ew')
        
        global_buttons = ttk.Frame(global_frame)
        global_buttons.grid(column=0, row=0, sticky='ew')
        global_buttons.columnconfigure((0, 1), weight=1)
        
        ttk.Button(global_buttons, text='Move Both to Neutral', command=self.move_both_to_neutral).grid(column=0, row=0, padx=4)
        ttk.Button(global_buttons, text='Open Both Grippers', command=lambda: self._send_gripper_both(0.04)).grid(column=1, row=0, padx=4)
        ttk.Button(global_buttons, text='Close Both Grippers', command=lambda: self._send_gripper_both(0.01)).grid(column=2, row=0, padx=4)

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

        # Status
        status_var = self.status_var_panda1 if arm == 'panda1' else self.status_var_panda2
        ttk.Label(parent, textvariable=status_var, foreground='gray').grid(column=0, row=7, columnspan=3, pady=(12, 0), sticky='w')

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

    # ------------------------------------------------------------------
    def _spin_loop(self) -> None:
        """ROS2 spin loop in separate thread."""
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            # Handle shutdown gracefully
            if 'ExternalShutdownException' not in str(type(e).__name__):
                self.node.get_logger().warn(f'Spin loop error: {e}')

    def _read_pose(self, arm: str) -> Pose:
        """Read pose from GUI entries for specified arm."""
        if arm == 'panda1':
            entries = (self.entry_x_p1, self.entry_y_p1, self.entry_z_p1,
                      self.entry_roll_p1, self.entry_pitch_p1, self.entry_yaw_p1)
            status_var = self.status_var_panda1
        else:
            entries = (self.entry_x_p2, self.entry_y_p2, self.entry_z_p2,
                      self.entry_roll_p2, self.entry_pitch_p2, self.entry_yaw_p2)
            status_var = self.status_var_panda2

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

        qx, qy, qz, qw = _euler_to_quaternion(roll, pitch, yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    # ------------------------------------------------------------------
    def move_to_pose(self, arm: str, execute: bool = True) -> None:
        """Move specified arm to target pose."""
        status_var = self.status_var_panda1 if arm == 'panda1' else self.status_var_panda2
        
        try:
            pose = self._read_pose(arm)
        except ValueError as exc:
            messagebox.showerror('Invalid input', str(exc))
            return

        status_var.set('Computing IK...')
        self.root.update_idletasks()

        try:
            joint_positions = self.node.compute_ik(pose, arm=arm)
        except RuntimeError as exc:
            status_var.set(f'IK failed: {exc}')
            return

        status_var.set('IK success. ' + ('Executing...' if execute else 'Not executing.'))
        self.root.update_idletasks()

        if execute:
            self.node.send_joint_trajectory(joint_positions, arm=arm)
            status_var.set('Command sent.')
        else:
            formatted = ', '.join(f'{val:.2f}' for val in joint_positions)
            status_var.set(f'Joint solution: [{formatted}]')

    def move_to_neutral(self, arm: str) -> None:
        """Move specified arm to neutral pose."""
        status_var = self.status_var_panda1 if arm == 'panda1' else self.status_var_panda2
        status_var.set('Moving to neutral...')
        self.node.send_joint_trajectory(self.node.neutral_pose, arm=arm)
        status_var.set('Neutral command sent.')

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

    # ------------------------------------------------------------------
    def run(self) -> None:
        """Start the GUI main loop."""
        self.root.mainloop()

    def shutdown(self) -> None:
        """Clean shutdown."""
        self.status_var_panda1.set('Closing...')
        self.status_var_panda2.set('Closing...')
        self.root.update_idletasks()

        # Stop the spin thread first
        if self.spin_thread.is_alive():
            # Give it a moment to finish current operation
            import time
            time.sleep(0.1)
        
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
        
        # Wait for thread to finish
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)

        self.root.destroy()


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


def main() -> None:
    """Main entry point."""
    gui = DualPandaIKGUI()
    try:
        gui.run()
    except KeyboardInterrupt:
        gui.shutdown()


if __name__ == '__main__':
    main()

