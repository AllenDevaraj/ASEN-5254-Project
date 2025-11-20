#!/bin/bash
# Script to check namespaced topics and services

echo "=== ALL TOPICS ==="
ros2 topic list

echo ""
echo "=== PANDA1 NAMESPACE TOPICS ==="
ros2 topic list | grep "^/panda1/"

echo ""
echo "=== PANDA2 NAMESPACE TOPICS ==="
ros2 topic list | grep "^/panda2/"

echo ""
echo "=== COMPUTE_IK SERVICES ==="
ros2 service list | grep compute_ik

echo ""
echo "=== MOVE_GROUP SERVICES ==="
ros2 service list | grep move_group

echo ""
echo "=== JOINT TRAJECTORY TOPICS ==="
ros2 topic list | grep joint_trajectory

echo ""
echo "=== GRIPPER ACTION SERVERS ==="
ros2 action list | grep gripper

