#!/usr/bin/env python3
"""
gripper_test.py  —  franka_bimanual_bringup/scripts/

Tests the Franka gripper in simulation using the FollowJointTrajectory action.
This is much more robust than raw topic publishing.

Action server: /franka1_gripper/follow_joint_trajectory
Action type:   control_msgs/action/FollowJointTrajectory

Usage:
  ros2 launch franka_bimanual_bringup gripper_test.launch.py arm:=right_arm
  ros2 launch franka_bimanual_bringup gripper_test.launch.py arm:=left_arm cycles:=5
"""

import argparse
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

MAX_FINGER_POS = 0.04   # metres (one finger)
MIN_FINGER_POS = 0.0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class GripperTestActionNode(Node):
    def __init__(self, arm: str, cycles: int, open_width: float, close_width: float, pause: float):
        super().__init__('gripper_test_action_node')
        
        if arm == 'left_arm':
            prefix = 'franka2'
        elif arm == 'right_arm':
            prefix = 'franka1'
        else:
            self.get_logger().error(f"❌ Unknown arm '{arm}'")
            sys.exit(1)

        action_name = f'/{prefix}_gripper/follow_joint_trajectory'
        self.joint_names = [f'{prefix}_fr3_finger_joint1', f'{prefix}_fr3_finger_joint2']
        
        self.open_pos = clamp(open_width / 2.0, MIN_FINGER_POS, MAX_FINGER_POS)
        self.close_pos = clamp(close_width / 2.0, MIN_FINGER_POS, MAX_FINGER_POS)
        self.pause = pause
        self.total_cycles = cycles
        
        self._client = ActionClient(self, FollowJointTrajectory, action_name)
        
        self.get_logger().info(f"🦾 Gripper Action Test | arm={arm} | action={action_name}")
        self.get_logger().info(f"⏳ Waiting for action server...")
        
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"❌ Action server {action_name} not found!")
            sys.exit(1)
            
        self.run_test()

    def run_test(self):
        for i in range(self.total_cycles):
            self.get_logger().info(f"── Cycle {i+1}/{self.total_cycles}: OPENING → {self.open_pos*2:.3f}m ──")
            self.send_goal(self.open_pos)
            
            self.get_logger().info(f"── Cycle {i+1}/{self.total_cycles}: CLOSING → {self.close_pos*2:.3f}m ──")
            self.send_goal(self.close_pos)

        self.get_logger().info("✅ Gripper test complete!")

    def send_goal(self, position):
        goal_msg = FollowJointTrajectory.Goal()
        
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start.sec = 1
        traj.points = [point]
        
        goal_msg.trajectory = traj
        
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected")
            return

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        import time
        time.sleep(self.pause)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--arm', default='right_arm')
    parser.add_argument('--cycles', type=int, default=3)
    parser.add_argument('--open', type=float, default=0.08)
    parser.add_argument('--close', type=float, default=0.0)
    parser.add_argument('--pause', type=float, default=1.0)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = GripperTestActionNode(args.arm, args.cycles, args.open, args.close, args.pause)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
