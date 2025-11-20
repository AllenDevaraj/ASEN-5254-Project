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
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DualPandaIKNode(Node):
    """ROS2 node handling IK requests and trajectory execution for both pandas."""

    def __init__(self) -> None:
        super().__init__('dual_panda_ik_gui_node')
        
        self.joint_names: List[str] = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7',
        ]
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

        # Status
        status_var = self.status_var_panda1 if arm == 'panda1' else self.status_var_panda2
        ttk.Label(parent, textvariable=status_var, foreground='gray').grid(column=0, row=6, columnspan=3, pady=(12, 0), sticky='w')

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

