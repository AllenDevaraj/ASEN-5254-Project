#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import math
import time
import threading
import sys

# --- Helper Functions ---
def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to Quaternion."""
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5
    cr = math.cos(half_roll); sr = math.sin(half_roll)
    cp = math.cos(half_pitch); sp = math.sin(half_pitch)
    cy = math.cos(half_yaw); sy = math.sin(half_yaw)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

# --- Class GazeboDetector (The Perception) ---
class GazeboDetector(Node):
    def __init__(self):
        super().__init__('gazebo_detector')
        self.poses = {}
        self.lock = threading.Lock()
        
        # Proper dimensions from SDF files
        self.dimensions = {
            'red_block': [0.05, 0.05, 0.05],
            'green_block': [0.05, 0.05, 0.05],
            'red_solid': [0.08, 0.03, 0.03],
            'green_solid': [0.08, 0.03, 0.03],
            'red_hollow': [0.08, 0.05, 0.05],
            'green_hollow': [0.08, 0.05, 0.05]
        }

        # Subscribe to Ground Truth topics (Gazebo Bridges)
        # NOTE: dual_panda_complete.launch.py must be running!
        for name in self.dimensions.keys():
            topic = f'/model/{name}/pose'
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, n=name: self._callback(msg, n),
                10
            )
        self.get_logger().info('GazeboDetector initialized. Listening for poses...')

    def _callback(self, msg, name):
        with self.lock:
            self.poses[name] = msg.pose

    def get_pose(self, entity_name):
        """Blocks until pose is available for the entity."""
        self.get_logger().info(f'Waiting for pose of {entity_name}...')
        while rclpy.ok():
            with self.lock:
                if entity_name in self.poses:
                    return self.poses[entity_name]
            time.sleep(0.1)
        return None

# --- Class PandaPickPlace (The Controller) ---
class PandaPickPlace(Node):
    def __init__(self, detector):
        super().__init__('panda_pick_place')
        self.detector = detector
        
        # Robot Config
        # 11cm offset from Wrist (panda_link8) to Fingertips (Grasp Point)
        self.gripper_length = 0.11 
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        # Setup Clients
        self.ik_client = self.create_client(GetPositionIK, '/panda1/compute_ik')
        self.cartesian_client = self.create_client(GetCartesianPath, '/panda1/compute_cartesian_path')
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/panda1/panda_arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/panda1/panda_gripper_controller/gripper_cmd')
        self.scene_pub = self.create_publisher(PlanningScene, '/panda1/monitored_planning_scene', 10)
        
        self.wait_for_services()
        self.setup_scene()

    def wait_for_services(self):
        self.get_logger().info('Waiting for MoveIt services...')
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('IK service not available.')
        self.cartesian_client.wait_for_service(timeout_sec=5.0)
        self.traj_client.wait_for_server(timeout_sec=5.0)
        self.gripper_client.wait_for_server(timeout_sec=5.0)
        self.get_logger().info('Services ready.')

    def setup_scene(self):
        # Add Table (Dimension: 1.0x2.0x0.2, Height: 0.2)
        scene = PlanningScene()
        scene.is_diff = True
        obj = CollisionObject()
        obj.id = 'table'
        obj.header.frame_id = 'world'
        obj.operation = CollisionObject.ADD
        
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [1.0, 3.0, 0.2] # From dual_arm_gui.py
        
        pose = Pose()
        pose.position.x = 0.7
        pose.position.y = 0.0
        pose.position.z = 0.1 # Center (0.2 height / 2)
        pose.orientation.w = 1.0
        
        obj.primitives = [prim]
        obj.primitive_poses = [pose]
        scene.world.collision_objects.append(obj)
        self.scene_pub.publish(scene)
        time.sleep(1.0)

    def move_ik(self, pose):
        """Solve IK and Execute Trajectory."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'panda_arm'
        req.ik_request.pose_stamped.header.frame_id = 'panda_link0'
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.avoid_collisions = True
        
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if res.error_code.val != res.error_code.SUCCESS:
            self.get_logger().error(f'IK Failed: {res.error_code.val}')
            return False
            
        joint_positions = list(res.solution.joint_state.position)
        # Filter joints (solution might contain finger joints)
        # Assuming simple mapping for now (first 7 are arm)
        joint_positions = joint_positions[:7]
        
        return self.execute_trajectory(joint_positions)

    def move_cartesian(self, waypoints):
        """Compute Cartesian Path and Execute."""
        req = GetCartesianPath.Request()
        req.header.frame_id = 'panda_link0'
        req.group_name = 'panda_arm'
        req.waypoints = waypoints
        req.max_step = 0.01
        req.avoid_collisions = True
        
        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if res.fraction < 0.9:
            self.get_logger().warn(f'Cartesian Path incomplete: {res.fraction*100}%')
            return False
            
        # Extract first point positions from solution
        traj = res.solution
        # We assume the controller can handle the full trajectory
        # But we need to send it via Action
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        
        return self.send_action_goal(goal)

    def execute_trajectory(self, positions):
        """Send simple point-to-point trajectory."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=3.0).to_msg()
        goal.trajectory.points = [point]
        return self.send_action_goal(goal)

    def send_action_goal(self, goal):
        """Blocking action call."""
        future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return False
            
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result()
        
        return result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def move_gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = width
        goal.command.max_effort = 20.0
        
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        # Gripper action doesn't always return standard error codes, assumed success

    def attach_object(self, name):
        scene = PlanningScene()
        scene.is_diff = True
        obj = AttachedCollisionObject()
        obj.link_name = 'panda_hand'
        obj.object.id = name
        obj.object.operation = CollisionObject.ADD # Or Attach logic logic
        # Note: AttachedCollisionObject adds to gripper, but we need to REMOVE from world too?
        # MoveIt handles this if we Attach.
        # Usually: Add CollisionObject (World), Then AttachedCollisionObject.
        # Here we just set touch links
        obj.touch_links = ['panda_hand', 'panda_leftfinger', 'panda_rightfinger']
        scene.robot_state.attached_collision_objects = [obj]
        self.scene_pub.publish(scene)

    def execute_pick(self, obj_name):
        self.get_logger().info(f'Starting pick for {obj_name}...')
        
        # 1. Get Pose
        pose_world = self.detector.get_pose(obj_name)
        if not pose_world:
            self.get_logger().error(f'Could not detect {obj_name}')
            return

        # 2. Orientation (Top-Down)
        # Object Yaw from Quaternion
        q = pose_world.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        obj_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Robot Base is at (0,0,0) for Panda1?
        # NOTE: In dual_panda_demo, Panda1 is at y=0.15?
        # We need to transform World Pose to Robot Frame (panda_link0).
        # Hardcoding transform for Panda1 (x=0, y=0.15, z=0)
        # If we assume Panda1 is at (0, 0.15, 0), Yaw=0.
        # Transform: x_local = x_world, y_local = y_world - 0.15
        
        # Let's compute local pose roughly
        local_pose = Pose()
        local_pose.position.x = pose_world.position.x
        local_pose.position.y = pose_world.position.y - 0.15
        local_pose.position.z = pose_world.position.z
        
        # Top Down Orientation (Roll=180) aligned with object yaw
        # Target Yaw = Object Yaw
        tx, ty, tz, tw = euler_to_quaternion(math.pi, 0.0, obj_yaw)
        local_pose.orientation.x = tx
        local_pose.orientation.y = ty
        local_pose.orientation.z = tz
        local_pose.orientation.w = tw
        
        # 3. Pre-Grasp (Hover)
        hover_pose = Pose()
        hover_pose.orientation = local_pose.orientation
        hover_pose.position.x = local_pose.position.x
        hover_pose.position.y = local_pose.position.y
        hover_pose.position.z = local_pose.position.z + 0.15 + self.gripper_length # TCP at +15cm from object center?
        # The prompt says "z = cube_pose.z + 0.15".
        # If cube_pose.z is center, then +0.15 puts center of gripper 15cm above center.
        # But gripper fingers extend down. 
        # TCP is wrist.
        # So wrist should be at (Center + 0.15 + GripperLength).
        # Wait, if prompt says "z = pose.z + 0.15", and we interpret pose as center.
        # Let's use: TCP Z = Center Z + 0.15 + 0.11 (Gripper)
        hover_pose.position.z = local_pose.position.z + 0.15 + self.gripper_length
        
        self.get_logger().info('Moving to Pre-Grasp...')
        self.move_ik(hover_pose)
        
        # 4. Linear Approach (Crucial: Re-Read)
        # Re-read pose in case it moved
        new_pose_world = self.detector.get_pose(obj_name)
        
        # Update Local X/Y
        approach_pose = Pose()
        approach_pose.orientation = local_pose.orientation # Keep orientation
        approach_pose.position.x = new_pose_world.position.x
        approach_pose.position.y = new_pose_world.position.y - 0.15
        
        # Target Z: Table Height + Size/2
        # "Table Height" (Top) = 0.2
        # Size/2 depends on object.
        # For Block: 0.025. Target Z (Center) = 0.225.
        # TCP Z = 0.225 + Gripper Length (0.11) = 0.335.
        # Using Ground Truth Z directly is better:
        center_z = new_pose_world.position.z
        approach_pose.position.z = center_z + self.gripper_length
        
        # Recalculate Hover at new X/Y for straight descent
        adjusted_hover = Pose()
        adjusted_hover.orientation = approach_pose.orientation
        adjusted_hover.position.x = approach_pose.position.x
        adjusted_hover.position.y = approach_pose.position.y
        adjusted_hover.position.z = hover_pose.position.z # Keep high Z
        
        self.get_logger().info('Adjusting and Descending...')
        self.move_cartesian([adjusted_hover, approach_pose])
        
        # 5. Grasp
        self.get_logger().info('Grasping...')
        self.move_gripper(0.0) # Close
        
        # 6. Attach
        self.attach_object(obj_name)
        
        # 7. Lift
        lift_pose = Pose()
        lift_pose.orientation = approach_pose.orientation
        lift_pose.position = approach_pose.position
        lift_pose.position.z += 0.15
        
        self.get_logger().info('Lifting...')
        self.move_cartesian([approach_pose, lift_pose])
        self.get_logger().info('Done.')

def main():
    rclpy.init()
    
    detector = GazeboDetector()
    controller = PandaPickPlace(detector)
    
    # Run Detector in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(detector,), daemon=True)
    spin_thread.start()
    
    # Wait a bit
    time.sleep(2.0)
    
    try:
        # Example: Pick red_solid
        controller.execute_pick('red_solid')
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

