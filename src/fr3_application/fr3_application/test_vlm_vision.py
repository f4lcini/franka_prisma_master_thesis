#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge

from franka_custom_interfaces.action import VlmQuery

class VlmTestClient(Node):
    def __init__(self):
        super().__init__('vlm_test_client')
        self._action_client = ActionClient(self, VlmQuery, 'vlm_query')
        self.cv_bridge = CvBridge()

    def send_goal(self, image_path, task_description):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = VlmQuery.Goal()
        goal_msg.task_description = task_description
        
        # Load image from disk
        self.get_logger().info(f'Loading image from: {image_path}')
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            self.get_logger().error(f"Failed to load image at {image_path}")
            return
            
        # Convert to ROS 2 Image msg
        ros_image = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        goal_msg.image_raw = ros_image

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.current_state}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('--- Result ---')
        self.get_logger().info(f'Success: {result.success}')
        self.get_logger().info(f'Target Label: {result.target_label}')
        self.get_logger().info(f'Action Choice: {result.action_choice}')
        self.get_logger().info(f'Selected Arm: {result.arm_selection}')
        self.get_logger().info(f'Handover Height (Z): {result.handover_height_z}m')
        self.get_logger().info(f'Reasoning:\n{result.reasoning}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = VlmTestClient()
    
    # Path to the workspace image
    image_path = "Images/Workspace.jpg"
    description = "Find the ball, decide which arm to use to pick it up, and place it inside the box. Also suggest a safe handover height if needed."
    
    action_client.send_goal(image_path, description)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
