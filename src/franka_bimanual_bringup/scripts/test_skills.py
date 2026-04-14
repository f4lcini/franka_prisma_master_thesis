#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_custom_interfaces.action import MoveHome, PickObject, PlaceObject
from geometry_msgs.msg import PoseStamped
import sys
import time

class SkillTester(Node):
    def __init__(self):
        super().__init__('skill_tester')
        self.home_client = ActionClient(self, MoveHome, 'move_home')
        self.pick_client = ActionClient(self, PickObject, 'pick_object')
        self.place_client = ActionClient(self, PlaceObject, 'place_object')

    def send_home(self, arm="left_arm"):
        self.get_logger().info(f"🏠 Sending HOME goal for {arm}...")
        if not self.home_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ Home Action Server not available!")
            return
        
        goal = MoveHome.Goal()
        goal.arm = arm
        return self.home_client.send_goal_async(goal)

    def send_pick(self, arm="left_arm", target="base_pose"):
        self.get_logger().info(f"📦 Sending PICK goal for {arm} ({target})...")
        if not self.pick_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Pick Action Server not available!")
            return
        
        goal = PickObject.Goal()
        goal.arm = arm
        
        # Predefined Target Logic (Server will override coords if frame_id matches)
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = target # "base_pose" or "shared"
        
        # Default coords if server doesn't override (Legacy Support)
        goal.target_pose.pose.position.x = 1.10
        goal.target_pose.pose.position.y = 0.20
        goal.target_pose.pose.position.z = 0.225
        
        # Standard Grasp Orientation (TCP pointing down)
        goal.target_pose.pose.orientation.x = 1.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 0.0
        
        return self.pick_client.send_goal_async(goal)

    def send_place(self, arm="left_arm", target="shared"):
        self.get_logger().info(f"📥 Sending PLACE goal for {arm} ({target})...")
        if not self.place_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Place Action Server not available!")
            return
        
        goal = PlaceObject.Goal()
        goal.arm = arm
        
        # Predefined Target Logic (Server will override coords if frame_id matches)
        goal.place_pose = PoseStamped()
        goal.place_pose.header.frame_id = target # "shared" or "box"
        
        # Default coords if server doesn't override (Legacy Support)
        goal.place_pose.pose.position.x = 0.50
        goal.place_pose.pose.position.y = 0.20
        goal.place_pose.pose.position.z = 0.30 
        
        return self.place_client.send_goal_async(goal)

def main():
    rclpy.init()
    tester = SkillTester()
    
    if len(sys.argv) < 2:
        print("Usage: python3 test_skills.py [home|pick|place] [left_arm|right_arm] [target]")
        return

    skill = sys.argv[1].lower()
    arm = sys.argv[2] if len(sys.argv) > 2 else "left_arm"
    target = sys.argv[3] if len(sys.argv) > 3 else None
    
    future = None
    if skill == "home":
        future = tester.send_home(arm)
    elif skill == "pick":
        # Default to base_pose if no target specified
        future = tester.send_pick(arm, target if target else "base_pose")
    elif skill == "place":
        # Default to shared if no target specified
        future = tester.send_place(arm, target if target else "shared")
    else:
        print(f"Unknown skill: {skill}")
        return

    if future:
        rclpy.spin_until_future_complete(tester, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal rejected")
            return
        
        print("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(tester, result_future)
        print(f"Result received: {result_future.result().result.success}")

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
