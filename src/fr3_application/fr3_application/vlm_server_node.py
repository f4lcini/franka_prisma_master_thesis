"""
================================================================================
Author: Falco Robotics (with AI Assistant)
Code Description: 
[ROLE]: ACTION SERVER (Provides: /vlm_query)

This ROS 2 node acts as a cognitive Action Server that integrates Google's Gemini 
Vision-Language Model (VLM). It takes a live camera image and a natural language 
task description from the client (e.g., the Behavior Tree), queries the VLM engine 
with rigorous kinematic constraints, and returns a structured JSON payload containing 
the parsed target object, the inferred primitive action, the selected robotic arm, 
and the reasoning behind it.

Pipeline: Reasoning -> Action Server

Implementation Steps Summary:
- NODE INITIALIZATION (Steps 1-2): Setup CvBridge, variable buffers, and the GenAI Client (requires GEMINI_API_KEY).
- TOPIC SUBSCRIBERS (Step 3): Start listening for the camera's RGB image stream into a reentrant callback group.
- ACTION SERVER (Step 4): Expose the 'vlm_query' Action Server to accept tasks from the Behavior Tree.
- IMAGE ACQUISITION (Step 5): Cache the latest image as a PIL object to be sent to Gemini.
- ACTION LIFECYCLE (Steps 6-7): Manage incoming goal acceptance and cancellations.
- GOAL EXECUTION (Step 8): Process the accepted goal, updating the feedback state continuously.
- PROMPT ENGINEERING (Step 9): Assemble the system prompt enforcing kinematic rules and handover dynamics.
- VLM INFERENCE (Step 10): Execute a blocking call to Gemini via the HTTP client passing the prompt and the captured image.
- DATA PARSING (Step 11): Use Pydantic to strictly validate the JSON output from Gemini.
- RESULT FINALIZATION (Step 12): Extract labels, action choices, and arm selection to populate and succeed the Action Result.
- MAIN LOOP & EXECUTOR (Steps 13-15): Spin the node using a MultiThreadedExecutor to keep callbacks alive during the API call.
================================================================================
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from franka_custom_interfaces.action import VlmQuery

from cv_bridge import CvBridge
import cv2
import PIL.Image

from fr3_application.skills_repertoire import TaskPlan
from google import genai
from google.genai import types

class VlmServerNode(Node):
    def __init__(self):
        super().__init__('vlm_server_node')
        
        self.get_logger().info("Initializing VLM Server Node (Planning Edition)...")
        
        # Step 1-2: Setup CvBridge, variable buffers, and the GenAI Client.
        self.cv_bridge = CvBridge()
        self.latest_image = None
        
        # Setup Gemini Client
        api_key = os.environ.get("GEMINI_API_KEY")
        if not api_key:
            self.get_logger().error("GEMINI_API_KEY environment variable not set. Node will fail to process requests.")
        self.gemini_client = genai.Client(api_key=api_key)
        self.model_name = "gemini-2.5-flash" # Use stable flash model with separate quota
        
        self.cb_group = ReentrantCallbackGroup()
        
        # Step 3: Start listening for the camera's RGB image stream into a reentrant callback group.
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.cb_group
        )
        
        # Step 4: Expose the 'vlm_query' Action Server to accept tasks from the Behavior Tree.
        self._action_server = ActionServer(
            self,
            VlmQuery,
            'vlm_query',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info("VLM Server Node Ready. Using TaskPlan Repertoire.")

    def image_callback(self, msg: Image):
        """Cache the latest image from the camera."""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.latest_image = PIL.Image.fromarray(cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def goal_callback(self, goal_request):
        """Accept or reject a new goal."""
        self.get_logger().info(f"Received new VLM query: '{goal_request.task_description}'")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept cancellations."""
        self.get_logger().info('Received cancel request for VLM query.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Process the goal using Gemini API + TaskPlan Repertoire."""
        self.get_logger().info('Executing VLM Task Planning...')
        
        feedback_msg = VlmQuery.Feedback()
        result = VlmQuery.Result()
        
        task_description = goal_handle.request.task_description
        
        # 1. Image preparation
        pil_image = None
        if self.latest_image is not None:
             feedback_msg.current_state = "Capturing workspace state..."
             goal_handle.publish_feedback(feedback_msg)
             pil_image = self.latest_image
        else:
             self.get_logger().warn("No camera frames received. Planning without visual context (Reduced accuracy).")

        # 2. Call Gemini API
        feedback_msg.current_state = "Reasoning about the sequence..."
        goal_handle.publish_feedback(feedback_msg)
        
        # System Prompt is now more concise as the rules are in the TaskPlan Pydantic descriptions
        system_prompt = (
            "You are the high-level cognitive planner for a dual-arm Franka robot. "
            "Your goal is to decompose the user's command into a logical sequence of robotic skills. "
            "Use the provided schema strictly. Analyze the image to determine object positions and arm availability."
        )
        
        contents = [system_prompt, f"User Command: {task_description}"]
        if pil_image:
            contents.append(pil_image)
            
        try:
            # Step 10: VLM Inference using structured output
            response = self.gemini_client.models.generate_content(
                model=self.model_name,
                contents=contents,
                config=types.GenerateContentConfig(
                    response_mime_type="application/json",
                    response_schema=TaskPlan,
                    temperature=0.1,
                ),
            )
            
            # 3. Return serialized JSON
            json_plan = response.text
            self.get_logger().info(f"Generated Plan: {json_plan}")
            
            result.success = True
            result.vlm_plan_json = json_plan
            result.message = "Task plan generated successfully."
            
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f"Gemini Planning Failed: {e}")
            result.success = False
            result.message = f"API Error: {str(e)}"
            goal_handle.abort()
            return result

def main(args=None):
    rclpy.init(args=args)
    node = VlmServerNode()
    
    # Step 13-15: Spin the node using a MultiThreadedExecutor to keep callbacks alive during the API call.
    # We use a MultiThreadedExecutor so the image callback can fire while execute_callback is waiting for Gemini
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
