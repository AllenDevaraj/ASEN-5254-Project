#!/usr/bin/env python3
"""
Joint State Combiner Node

Merges joint states from /panda1/joint_states and /panda2/joint_states
into a single /joint_states topic with prefixed joint names (panda1_*, panda2_*).
This is required for unified MoveIt move_group.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading


class JointStateCombiner(Node):
    """Combines joint states from two robots with prefixes."""
    
    def __init__(self):
        super().__init__('joint_state_combiner')
        
        # Subscribers for individual robot joint states
        self.panda1_sub = self.create_subscription(
            JointState,
            '/panda1/joint_states',
            lambda msg: self._joint_state_callback(msg, 'panda1'),
            10
        )
        
        self.panda2_sub = self.create_subscription(
            JointState,
            '/panda2/joint_states',
            lambda msg: self._joint_state_callback(msg, 'panda2'),
            10
        )
        
        # Publisher for combined joint states
        self.combined_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Store latest joint states
        self.lock = threading.Lock()
        self.panda1_state = None
        self.panda2_state = None
        
        # Timer to publish combined states
        self.timer = self.create_timer(0.05, self._publish_combined)  # 20 Hz
        
        self.get_logger().info('Joint State Combiner initialized')
    
    def _joint_state_callback(self, msg: JointState, robot: str):
        """Store joint state from one robot."""
        with self.lock:
            if robot == 'panda1':
                self.panda1_state = msg
            else:
                self.panda2_state = msg
    
    def _publish_combined(self):
        """Combine and publish joint states with prefixes."""
        with self.lock:
            if self.panda1_state is None and self.panda2_state is None:
                return
            
            # Create combined joint state
            combined = JointState()
            combined.header.stamp = self.get_clock().now().to_msg()
            combined.header.frame_id = ''  # No frame
            
            # Add Panda 1 joints with prefix
            if self.panda1_state is not None:
                for i, name in enumerate(self.panda1_state.name):
                    # Replace "panda_" with "panda1_" (not prepend, to avoid panda1_panda_joint7)
                    if name.startswith('panda_'):
                        prefixed_name = name.replace('panda_', 'panda1_', 1)
                    elif name.startswith('panda1_'):
                        prefixed_name = name  # Already prefixed
                    else:
                        prefixed_name = f'panda1_{name}'  # Fallback: prepend
                    
                    combined.name.append(prefixed_name)
                    if i < len(self.panda1_state.position):
                        combined.position.append(self.panda1_state.position[i])
                    if i < len(self.panda1_state.velocity):
                        combined.velocity.append(self.panda1_state.velocity[i])
                    if i < len(self.panda1_state.effort):
                        combined.effort.append(self.panda1_state.effort[i])
            
            # Add mimic finger_joint2 for Panda 1 (if finger_joint1 exists)
            finger1_idx_p1 = None
            if self.panda1_state is not None:
                for i, name in enumerate(self.panda1_state.name):
                    if name == 'panda_finger_joint1':
                        finger1_idx_p1 = i
                        break
            if finger1_idx_p1 is not None:
                # Add finger_joint2 as mimic (copy values from finger_joint1)
                combined.name.append('panda1_finger_joint2')
                if finger1_idx_p1 < len(self.panda1_state.position):
                    combined.position.append(self.panda1_state.position[finger1_idx_p1])
                else:
                    combined.position.append(0.0)
                if finger1_idx_p1 < len(self.panda1_state.velocity):
                    combined.velocity.append(self.panda1_state.velocity[finger1_idx_p1])
                else:
                    combined.velocity.append(0.0)
                if finger1_idx_p1 < len(self.panda1_state.effort):
                    combined.effort.append(self.panda1_state.effort[finger1_idx_p1])
                else:
                    combined.effort.append(0.0)
            
            # Add Panda 2 joints with prefix
            if self.panda2_state is not None:
                for i, name in enumerate(self.panda2_state.name):
                    # Replace "panda_" with "panda2_" (not prepend, to avoid panda2_panda_joint7)
                    if name.startswith('panda_'):
                        prefixed_name = name.replace('panda_', 'panda2_', 1)
                    elif name.startswith('panda2_'):
                        prefixed_name = name  # Already prefixed
                    else:
                        prefixed_name = f'panda2_{name}'  # Fallback: prepend
                    
                    combined.name.append(prefixed_name)
                    # Append values (they'll align with names)
                    if i < len(self.panda2_state.position):
                        combined.position.append(self.panda2_state.position[i])
                    elif len(combined.position) < len(combined.name):
                        combined.position.append(0.0)  # Fill missing
                    if i < len(self.panda2_state.velocity):
                        combined.velocity.append(self.panda2_state.velocity[i])
                    elif len(combined.velocity) < len(combined.name):
                        combined.velocity.append(0.0)  # Fill missing
                    if i < len(self.panda2_state.effort):
                        combined.effort.append(self.panda2_state.effort[i])
            
            # Add mimic finger_joint2 for Panda 2 (if finger_joint1 exists)
            finger1_idx_p2 = None
            if self.panda2_state is not None:
                for i, name in enumerate(self.panda2_state.name):
                    if name == 'panda_finger_joint1':
                        finger1_idx_p2 = i
                        break
            if finger1_idx_p2 is not None:
                # Add finger_joint2 as mimic (copy values from finger_joint1)
                combined.name.append('panda2_finger_joint2')
                if finger1_idx_p2 < len(self.panda2_state.position):
                    combined.position.append(self.panda2_state.position[finger1_idx_p2])
                else:
                    combined.position.append(0.0)
                if finger1_idx_p2 < len(self.panda2_state.velocity):
                    combined.velocity.append(self.panda2_state.velocity[finger1_idx_p2])
                else:
                    combined.velocity.append(0.0)
                if finger1_idx_p2 < len(self.panda2_state.effort):
                    combined.effort.append(self.panda2_state.effort[finger1_idx_p2])
                else:
                    combined.effort.append(0.0)
            
            # Publish combined state
            self.combined_pub.publish(combined)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateCombiner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

