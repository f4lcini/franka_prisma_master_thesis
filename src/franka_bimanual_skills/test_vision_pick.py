#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_custom_interfaces.action import DetectObject, PickObject
from geometry_msgs.msg import PoseStamped

class VisionPickTest(Node):
    def __init__(self):
        super().__init__('vision_pick_test_node')
        
        self.detect_client = ActionClient(self, DetectObject, 'detect_object')
        self.pick_client = ActionClient(self, PickObject, 'pick_object')
        
        self.get_logger().info('Vision Pick Test Node Ready.')

    async def run_test(self, object_name='bottle', arm='left_arm'):
        # 1. WAIT FOR SERVERS
        self.get_logger().info(f'Waiting for actions... (Detect & Pick)')
        self.detect_client.wait_for_server()
        self.pick_client.wait_for_server()

        # 2. DETECT OBJECT
        self.get_logger().info(f'--- STEP 1: Detecting {object_name} ---')
        detect_goal = DetectObject.Goal()
        detect_goal.object_name = object_name
        
        detect_handle = await self.detect_client.send_goal_async(detect_goal)
        if not detect_handle.accepted:
            self.get_logger().error('Detection goal rejected!')
            return

        detect_result = await detect_handle.get_result_async()
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
        
        pick_handle = await self.pick_client.send_goal_async(pick_goal)
        if not pick_handle.accepted:
            self.get_logger().error('Pick goal rejected!')
            return

        self.get_logger().info('Executing pick...')
        pick_result = await pick_handle.get_result_async()
        
        if pick_result.result.success:
            self.get_logger().info('🎉 SUCCESS: Object picked!')
        else:
            self.get_logger().error(f'❌ FAILED: {pick_result.result.message}')

async def main():
    rclpy.init()
    node = VisionPickTest()
    
    # PARAMETERS: Using left arm as requested
    await node.run_test(object_name='bottle', arm='left_arm')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import asyncio
    asyncio.run(main())
