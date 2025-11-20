#!/usr/bin/env python3

"""Tkinter GUI to command Panda end-effector poses via MoveIt's IK service."""

import argparse
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
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PandaIKNode(Node):
    """ROS2 node handling IK requests and trajectory execution."""

    def __init__(
        self,
        namespace: str = '',
        base_frame: str = 'panda_link0',
        move_group: str = 'panda_arm',
    ) -> None:
        self.namespace = namespace.strip('/')
        node_name = f'{self.namespace}_panda_ik_gui_node' if self.namespace else 'panda_ik_gui_node'
        super().__init__(node_name)
        self.base_frame = base_frame
        self.move_group = move_group

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

        self.joint_pub = self.create_publisher(
            JointTrajectory,
            self._ns_topic('panda_arm_controller/joint_trajectory'),
            10,
        )

        ik_service_name = self._ns_topic('compute_ik')
        self.ik_client = self.create_client(GetPositionIK, ik_service_name)
        self.get_logger().info(f'Waiting for {ik_service_name} service...')
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'MoveIt {ik_service_name} service not available.')

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            self._ns_topic('panda_gripper_controller/gripper_cmd'),
        )
        self.get_logger().info('Waiting for gripper action server...')
        self.gripper_client.wait_for_server(timeout_sec=5.0)

    # ------------------------------------------------------------------
    def _ns_topic(self, relative: str) -> str:
        relative = relative.lstrip('/')
        if self.namespace:
            return f'/{self.namespace}/{relative}'
        return f'/{relative}'

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------
    def compute_ik(self, pose: Pose) -> List[float]:
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.move_group
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=2.0).to_msg()

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.pose = pose
        request.ik_request.pose_stamped = pose_stamped

        future = self.ik_client.call_async(request)
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

    def send_joint_trajectory(self, joint_positions: List[float], seconds: float = 3.0) -> None:
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
        traj.points.append(point)

        self.joint_pub.publish(traj)

    def send_gripper_goal(self, width: float) -> None:
        goal = GripperCommand.Goal()
        goal.command.position = width
        goal.command.max_effort = 20.0

        if not self.gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Gripper action server unavailable.')
            return

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error('Failed to send gripper goal (no goal handle).')
            return
        if not goal_handle.accepted:
            self.get_logger().warn('Gripper goal rejected.')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)


class PandaIKGUI:
    """Tkinter front-end wrapping PandaIKNode."""

    def __init__(
        self,
        namespace: str = '',
        base_frame: str = 'panda_link0',
        move_group: str = 'panda_arm',
    ) -> None:
        rclpy.init()
        self.node = PandaIKNode(namespace=namespace, base_frame=base_frame, move_group=move_group)
        self.namespace = namespace.strip('/')

        self.root = tk.Tk()
        title_suffix = f' ({self.namespace})' if self.namespace else ''
        self.root.title(f'Panda End-Effector IK Demo{title_suffix}')
        self.root.protocol('WM_DELETE_WINDOW', self.shutdown)

        self.status_var = tk.StringVar(value='Ready')
        self._build_layout()

        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.spin_thread.start()

    # ------------------------------------------------------------------
    def _build_layout(self) -> None:
        main_frame = ttk.Frame(self.root, padding=12)
        main_frame.grid(column=0, row=0, sticky='nsew')
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        ttk.Label(main_frame, text='Target Position (meters)').grid(column=0, row=0, columnspan=3, pady=(0, 4))
        self.entry_x = self._make_entry(main_frame, 'X', 0.5, column=0, row=1)
        self.entry_y = self._make_entry(main_frame, 'Y', 0.0, column=1, row=1)
        self.entry_z = self._make_entry(main_frame, 'Z', 0.4, column=2, row=1)

        ttk.Label(main_frame, text='Target Orientation (radians)').grid(column=0, row=2, columnspan=3, pady=(12, 4))
        self.entry_roll = self._make_entry(main_frame, 'Roll', math.pi, column=0, row=3)
        self.entry_pitch = self._make_entry(main_frame, 'Pitch', 0.0, column=1, row=3)
        self.entry_yaw = self._make_entry(main_frame, 'Yaw', 0.0, column=2, row=3)

        buttons = ttk.Frame(main_frame)
        buttons.grid(column=0, row=4, columnspan=3, pady=(16, 8), sticky='ew')
        buttons.columnconfigure((0, 1, 2), weight=1)
        ttk.Button(buttons, text='Move', command=self.move_to_pose).grid(column=0, row=0, padx=4)
        ttk.Button(buttons, text='Plan Only', command=lambda: self.move_to_pose(execute=False)).grid(column=1, row=0, padx=4)
        ttk.Button(buttons, text='Neutral', command=self.move_to_neutral).grid(column=2, row=0, padx=4)

        grip = ttk.Frame(main_frame)
        grip.grid(column=0, row=5, columnspan=3, sticky='ew')
        grip.columnconfigure((0, 1), weight=1)
        ttk.Button(grip, text='Open Gripper', command=lambda: self._send_gripper(0.04)).grid(column=0, row=0, padx=4, pady=(0, 4))
        ttk.Button(grip, text='Close Gripper', command=lambda: self._send_gripper(0.01)).grid(column=1, row=0, padx=4, pady=(0, 4))

        ttk.Label(main_frame, textvariable=self.status_var, foreground='gray').grid(column=0, row=6, columnspan=3, pady=(12, 0), sticky='w')

    def _make_entry(self, frame: ttk.Frame, label: str, default: float, column: int, row: int) -> tk.Entry:
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
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _read_pose(self) -> Pose:
        try:
            x = float(self.entry_x.get())
            y = float(self.entry_y.get())
            z = float(self.entry_z.get())
            roll = float(self.entry_roll.get())
            pitch = float(self.entry_pitch.get())
            yaw = float(self.entry_yaw.get())
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
    def move_to_pose(self, execute: bool = True) -> None:
        try:
            pose = self._read_pose()
        except ValueError as exc:
            messagebox.showerror('Invalid input', str(exc))
            return

        self.status_var.set('Computing IK...')
        self.root.update_idletasks()

        try:
            joint_positions = self.node.compute_ik(pose)
        except RuntimeError as exc:
            self.status_var.set(f'IK failed: {exc}')
            return

        self.status_var.set('IK success. ' + ('Executing...' if execute else 'Not executing.'))
        self.root.update_idletasks()

        if execute:
            self.node.send_joint_trajectory(joint_positions)
            self.status_var.set('Command sent.')
        else:
            formatted = ', '.join(f'{val:.2f}' for val in joint_positions)
            self.status_var.set(f'Joint solution: [{formatted}]')

    def move_to_neutral(self) -> None:
        self.status_var.set('Moving to neutral...')
        self.node.send_joint_trajectory(self.node.neutral_pose)
        self.status_var.set('Neutral command sent.')

    def _send_gripper(self, width: float) -> None:
        self.status_var.set('Sending gripper command...')
        self.node.send_gripper_goal(width)
        self.status_var.set('Gripper command sent.')

    # ------------------------------------------------------------------
    def run(self) -> None:
        self.root.mainloop()

    def shutdown(self) -> None:
        self.status_var.set('Closing...')
        self.root.update_idletasks()

        rclpy.shutdown()
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)

        self.node.destroy_node()
        self.root.destroy()


def run_gui(namespace: str = '', base_frame: str = 'panda_link0', move_group: str = 'panda_arm') -> None:
    gui = PandaIKGUI(namespace=namespace, base_frame=base_frame, move_group=move_group)
    try:
        gui.run()
    except KeyboardInterrupt:
        gui.shutdown()


def main(argv: List[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description='Panda end-effector IK GUI')
    parser.add_argument('-n', '--namespace', default='', help='ROS namespace for this Panda robot')
    parser.add_argument('--base-frame', default='panda_link0', help='Base frame used for IK requests')
    parser.add_argument('--planning-group', default='panda_arm', help='MoveIt planning group name')
    args = parser.parse_args(args=argv)

    run_gui(namespace=args.namespace, base_frame=args.base_frame, move_group=args.planning_group)


def panda1_main() -> None:
    run_gui(namespace='panda1')


def panda2_main() -> None:
    run_gui(namespace='panda2')


if __name__ == '__main__':
    main()


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
