#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_custom_interfaces.action import PickObject
from geometry_msgs.msg import PoseStamped
import sys

class StandalonePickTester(Node):
    def __init__(self, arm="right_arm"):
        super().__init__('standalone_pick_tester')
        self._action_client = ActionClient(self, PickObject, 'pick_object')
        self.arm = arm

    def send_goal(self):
        goal_msg = PickObject.Goal()
        goal_msg.arm = self.arm
        
        # Target: Red Cube Pose from World File
        target = PoseStamped()
        target.header.frame_id = "world"
        target.pose.position.x = 1.1
        target.pose.position.y = 0.2
        target.pose.position.z = 0.225
        
        goal_msg.target_pose = target
        
        self.get_logger().info(f"🚀 Sending Standalone PICK Goal for {self.arm}...")
        self.get_logger().info("🎯 Target: [1.1, 0.2, 0.225] in world frame")

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal REJECTED")
            return

        self.get_logger().info("✅ Goal ACCEPTED")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"📊 [Feedback]: {feedback.status} ({feedback.completion_percentage:.1f}%)")

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"🏁 SUCCESS: {result.message}")
        else:
            self.get_logger().error(f"❌ FAILED: {result.message}")
        
        rclpy.shutdown()

def main():
    rclpy.init()
    arm = "right_arm"
    if len(sys.argv) > 1:
        arm = sys.argv[1]
        
    tester = StandalonePickTester(arm=arm)
    tester.send_goal()
    rclpy.spin(tester)

if __name__ == '__main__':
    main()
