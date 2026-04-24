#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_custom_interfaces.action import DetectObject, PickObject
from geometry_msgs.msg import PoseStamped
import sys

class VisionPickTest(Node):
    def __init__(self):
        super().__init__('vision_pick_test_node')
        
        self.detect_client = ActionClient(self, DetectObject, 'detect_object')
        self.pick_client = ActionClient(self, PickObject, 'pick_object')
        
        self.get_logger().info('Vision Pick Test Node Ready.')

    def run_test(self, object_name='bottle', arm='left_arm'):
        # 1. WAIT FOR SERVERS
        self.get_logger().info(f'Waiting for actions... (Detect & Pick)')
        if not self.detect_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Detect Action Server not found!')
            return
        if not self.pick_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Pick Action Server not found!')
            return

        # 2. DETECT OBJECT
        self.get_logger().info(f'--- STEP 1: Detecting {object_name} ---')
        detect_goal = DetectObject.Goal()
        detect_goal.object_name = object_name
        
        detect_future = self.detect_client.send_goal_async(detect_goal)
        rclpy.spin_until_future_complete(self, detect_future)
        
        detect_handle = detect_future.result()
        if not detect_handle.accepted:
            self.get_logger().error('Detection goal rejected!')
            return

        result_future = detect_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        detect_result = result_future.result()
        if not detect_result.result.success:
            self.get_logger().error('Detection failed!')
            return

        target_pose = detect_result.result.target_pose
        self.get_logger().info(f'✅ Found {object_name} at: {target_pose.pose.position}')

        # 3. PICK OBJECT
        self.get_logger().info(f'--- STEP 2: Picking with {arm} ---')
        pick_goal = PickObject.Goal()
        pick_goal.arm = arm
        pick_goal.target_pose = target_pose # Passing the pose from YOLO
        
        pick_future = self.pick_client.send_goal_async(pick_goal)
        rclpy.spin_until_future_complete(self, pick_future)
        
        pick_handle = pick_future.result()
        if not pick_handle.accepted:
            self.get_logger().error('Pick goal rejected!')
            return

        self.get_logger().info('Executing pick...')
        result_future_pick = pick_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future_pick)
        
        pick_result = result_future_pick.result()
        
        if pick_result.result.success:
            self.get_logger().info('🎉 SUCCESS: Object picked!')
        else:
            self.get_logger().error(f'❌ FAILED: {pick_result.result.message}')

def main():
    rclpy.init()
    node = VisionPickTest()
    
    try:
        # Using left_arm as default for this test
        node.run_test(object_name='bottle', arm='left_arm')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
