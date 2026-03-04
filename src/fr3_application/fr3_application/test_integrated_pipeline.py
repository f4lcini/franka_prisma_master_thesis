#!/usr/bin/env python3

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
        
        # Clients for both action servers
        self.vlm_client = ActionClient(self, VlmQuery, 'vlm_query')
        self.loc_client = ActionClient(self, DetectObject, 'detect_object')
        self.cv_bridge = CvBridge()
        # Definisco i client per la vlm e la localization
        # We store partial results here
        self.vlm_result = None

    def start_test(self, task_description, image_path=None):
        """
        Step 1: Ask VLM what to do.
        """
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
        result = future.result().result
        self.vlm_result = result
        
        self.get_logger().info("--- VLM DECISION RECEIVED ---")
        self.get_logger().info(f"Target Object: {result.target_label}")
        self.get_logger().info(f"Action:        {result.action_choice}")
        self.get_logger().info(f"Selected Arm:  {result.arm_selection}")
        self.get_logger().info(f"Reasoning:\n{result.reasoning}")
        self.get_logger().info("-----------------------------")

        if result.success and result.target_label.lower() != "none":
            # Proceed to Step 2
            self.locate_target(result.target_label)
        else:
            self.get_logger().warn("VLM returned 'none' as target or failed. Test finished.")
            rclpy.shutdown()

    def locate_target(self, target_label):
        """
        Step 2: Ask Object Localization for X, Y, Z coordinates.
        """
        self.get_logger().info(f"\n--- STEP 2: Asking YOLO + Depth to locate '{target_label}' ---")
        self.loc_client.wait_for_server()
        
        goal_msg = DetectObject.Goal()
        goal_msg.object_name = target_label
        
        future = self.loc_client.send_goal_async(goal_msg)
        future.add_done_callback(self.loc_goal_response_callback)

    def loc_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Localization Goal rejected.")
            rclpy.shutdown()
            return
            
        self.get_logger().info("Localization Goal accepted! Running YOLO and PointCloud extraction...")
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.loc_result_callback)

    def loc_result_callback(self, future):
        result = future.result().result
        
        self.get_logger().info("--- LOCALIZATION RESULT RECEIVED ---")
        if result.success:
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
    test_node = IntegratedPerceptionTest()
    
    # You can change the command here or pass an image path if the camera is off
    command = "Pick up the red cube and pass it to the other arm."
    image = None # E.g., "Images/Workspace.jpg"
    
    test_node.start_test(task_description=command, image_path=image)
    rclpy.spin(test_node)

if __name__ == '__main__':
    main()
