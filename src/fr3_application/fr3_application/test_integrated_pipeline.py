#!/usr/bin/env python3

"""
================================================================================
Author: Falco Robotics 
Code Description: 
This CLI-based ROS 2 testing node verifies the integration and handshaking between 
the High-Level VLM task planner (vlm_server_node) and the Mid-Level 3D Object 
Localization module (object_localization_node). It sends an asynchronous text/image 
query to the VLM Action Server, parses the VLM's string output, and immediately feeds 
the target object name into the Object Localization Action Server to test End-To-End extraction.

Pipeline: End-To-End Testing

Implementation Steps Summary:
- NODE INITIALIZATION (Steps 1-2): Setup testing node and the respective ActionClients for the 'vlm_query' and 'detect_object' servers.
- STEP 1 (VLM INITIATION) (Steps 3-4): Format the testing prompt and trigger the Gemini VLM goal asymptomatically.
- VLM CALLBACK (Steps 5-6): Wait for the goal to be accepted, process the reasoning feedback, and extract the JSON validation struct.
- STEP 2 (LOCALIZATION CALL) (Steps 7-8): If a valid target_label is returned by the VLM, pass it as the target name to the localization pipeline.
- LOCALIZATION CALLBACK (Steps 9-10): Wait for the depth-projected coordinates of the bounding box.
- TEST SUMMARY (Step 11): Intercept the final Z, Y, X pose from the payload and format a user-friendly console dashboard logging all pipeline outcomes.
- MAIN EXECUTION (Steps 12-13): Hardcode the testing prompt and run the unified asynchronous sequence.
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import cv2
from cv_bridge import CvBridge

from franka_custom_interfaces.action import VlmQuery, DetectObject

class IntegratedPerceptionTest(Node):
    def __init__(self):
        super().__init__('integrated_perception_test')
        self.get_logger().info("Initiating Test Node for VLM and Localization...")
        
        # Step 1-2: Setup testing node and the respective ActionClients for the 'vlm_query' and 'detect_object' servers.
        self.vlm_client = ActionClient(self, VlmQuery, 'vlm_query')
        self.loc_client = ActionClient(self, DetectObject, 'detect_object')
        self.cv_bridge = CvBridge()
        
        # We store partial results here
        self.vlm_result = None

    def start_test(self, task_description, image_path=None):
        # Step 3-4: Format the testing prompt and trigger the Gemini VLM goal asymptomatically.
        self.get_logger().info("--- STEP 1: Sending Task to VLM ---")
        self.get_logger().info(f"Task: '{task_description}'")
        
        self.vlm_client.wait_for_server()
        
        goal_msg = VlmQuery.Goal()
        goal_msg.task_description = task_description
        
        if image_path:
            cv_image = cv2.imread(image_path)
            if cv_image is not None:
                goal_msg.image_raw = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                
        # Send the goal asynchronously
        future = self.vlm_client.send_goal_async(goal_msg)
        future.add_done_callback(self.vlm_goal_response_callback)

    def vlm_goal_response_callback(self, future):
        # Step 5-6 part 1: Wait for the goal to be accepted...
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("VLM Goal rejected.")
            rclpy.shutdown()
            return

        self.get_logger().info("VLM Goal accepted! Waiting for reasoning...")
        # Now wait for the result
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.vlm_result_callback)

    def vlm_result_callback(self, future):
        # Step 5-6 part 2: ...process the reasoning feedback, and extract the JSON validation struct.
        result = future.result().result
        self.vlm_result = result
        
        self.get_logger().info("--- VLM DECISION RECEIVED ---")
        self.get_logger().info(f"Target Object: {result.target_label}")
        self.get_logger().info(f"Action:        {result.action_choice}")
        self.get_logger().info(f"Selected Arm:  {result.arm_selection}")
        self.get_logger().info(f"Reasoning:\n{result.reasoning}")
        self.get_logger().info("-----------------------------")

        # Step 7-8: If a valid target_label is returned by the VLM, pass it as the target name to the localization pipeline.
        if result.success and result.target_label.lower() != "none":
            self.locate_target(result.target_label)
        else:
            self.get_logger().warn("VLM returned 'none' as target or failed. Test finished.")
            rclpy.shutdown()

    def locate_target(self, target_label):
        self.get_logger().info(f"\n--- STEP 2: Asking YOLO + Depth to locate '{target_label}' ---")
        self.loc_client.wait_for_server()
        
        goal_msg = DetectObject.Goal()
        goal_msg.object_name = target_label
        
        future = self.loc_client.send_goal_async(goal_msg)
        future.add_done_callback(self.loc_goal_response_callback)

    def loc_goal_response_callback(self, future):
        # Step 9-10 part 1: Wait for the depth-projected coordinates...
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Localization Goal rejected.")
            rclpy.shutdown()
            return
            
        self.get_logger().info("Localization Goal accepted! Running YOLO and PointCloud extraction...")
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.loc_result_callback)

    def loc_result_callback(self, future):
        # Step 9-10 part 2: ...of the bounding box.
        result = future.result().result
        
        self.get_logger().info("--- LOCALIZATION RESULT RECEIVED ---")
        if result.success:
            # Step 11: Intercept the final Z, Y, X pose from the payload and format a user-friendly console dashboard logging all pipeline outcomes.
            p = result.target_pose.pose.position
            self.get_logger().info(f"SUCCESS! Target found.")
            self.get_logger().info(f"World Coordinates (frame: {result.target_pose.header.frame_id}):")
            self.get_logger().info(f"  X: {p.x:.3f} m")
            self.get_logger().info(f"  Y: {p.y:.3f} m")
            self.get_logger().info(f"  Z: {p.z:.3f} m")
            
            self.get_logger().info("\n==============================================")
            self.get_logger().info("TEST SUMMARY (What gets sent to moveit/planner):")
            self.get_logger().info(f"EXECUTE: {self.vlm_result.action_choice.upper()}")
            self.get_logger().info(f"ROBOT ARM: {self.vlm_result.arm_selection.upper()}")
            self.get_logger().info(f"DESTINATION: (x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f})")
            self.get_logger().info("==============================================")
            
        else:
            self.get_logger().error(f"Localization failed. Reason: {result.message} {result.error_message}")
            
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    import sys
    
    parser = argparse.ArgumentParser(description='Test Integrated Perception Pipeline')
    parser.add_argument('command', type=str, nargs='?', default="Pick up the red cube and pass it to the other arm.", help='The task description for the VLM')
    parser.add_argument('--image', type=str, default=None, help='Path to an offline image (optional, uses camera if omitted)')
    
    # Parse args, removing ROS 2 specific args
    args_without_ros = rclpy.utilities.remove_ros_args(args=sys.argv)
    parsed_args = parser.parse_args(args_without_ros[1:])
    
    test_node = IntegratedPerceptionTest()
    
    # Use the parsed command and image path
    test_node.start_test(task_description=parsed_args.command, image_path=parsed_args.image)
    rclpy.spin(test_node)

if __name__ == '__main__':
    main()
