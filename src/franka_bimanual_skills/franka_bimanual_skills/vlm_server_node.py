"""
================================================================================
Author: Falco Robotics 
Code Description: 
[ROLE]: ACTION SERVER (Provides: /vlm_query)
Generalized Bimanual Orchestrator version.
================================================================================
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from sensor_msgs.msg import Image
from franka_custom_interfaces.action import VlmQuery

from cv_bridge import CvBridge
import cv2
import PIL.Image

from franka_bimanual_skills.skills_repertoire import TaskPlan
from google import genai
from google.genai import types

class VlmServerNode(Node):
    def __init__(self):
        super().__init__('vlm_server_node')
        
        self.get_logger().info("Initializing Generalized VLM Bimanual Orchestrator...")
        
        self.cv_bridge = CvBridge()
        self.latest_image = None
        
        api_key = os.environ.get("GEMINI_API_KEY")
        if not api_key:
            self.get_logger().error("GEMINI_API_KEY environment variable not set.")
        self.gemini_client = genai.Client(api_key=api_key)
        self.model_name = "gemini-2.5-flash"
        
        self.cb_group = ReentrantCallbackGroup()
        
        # ---- Parameters ----
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('use_sensor_data_qos', False)
        
        image_topic = self.get_parameter('image_topic').value
        use_sensor_qos = self.get_parameter('use_sensor_data_qos').value
        
        # QoS Strategy: Best Effort for real hardware, Reliable for simulation
        if use_sensor_qos:
            self.get_logger().info("Using SensorDataQoS (Best Effort) for lab hardware.")
            qos = qos_profile_sensor_data
        else:
            self.get_logger().info("Using Reliable QoS for simulation.")
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            )

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos,
            callback_group=self.cb_group
        )
        
        self._action_server = ActionServer(
            self,
            VlmQuery,
            'vlm_query',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info("VLM Bimanual Orchestrator Ready.")

    def image_callback(self, msg: Image):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.latest_image = PIL.Image.fromarray(cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        feedback_msg = VlmQuery.Feedback()
        result = VlmQuery.Result()
        task_description = goal_handle.request.task_description
        
        # GENERALIZED BIMANUAL PROMPT
        system_prompt = (
            "You are the Master Orchestrator for a dual-arm Franka Research 3 robot system. "
            "Your goal is to provide a SMOOTH, PARALLEL, and COORDINATED plan for any task. "
            "The system executes two arm lanes SIMULTANEOUSLY. "
            "PERCEPTION: YOLO is ONLINE. To pick a variable object (like 'red_cube'), you MUST "
            "first call 'FIND_OBJECT' to update the blackboard pose. "
            "PREDEFINED FRAMES: 'shared' (relay), 'box' (destination). "
            "TOPOLOGY (CAMERA PERSPECTIVE): "
            "- 'right_arm': Appears on the LEFT side of the image. Reaches 'base_pose' and 'shared'. "
            "- 'left_arm': Appears on the RIGHT side of the image. Reaches 'shared' and 'box'. "
            "- 'box' is unreachable by 'right_arm'. 'base_pose' is unreachable by 'left_arm'. "
            "COORDINATION STRATEGY: "
            "1. PARALLELISM: Whenever possible, assign actions to BOTH arms starting from the first step. "
            "2. MID-AIR HANDOVER: To move an object from base_pose to box, use a direct exchange: "
            "   - Donor Arm (GIVE): Picks from source, then 'GIVE' at 'mid_air'. "
            "   - Recipient Arm (TAKE): Starts 'TAKE' at 'mid_air', then places at 'box'. "
            "3. SMOOTHNESS: Avoid idle arms. Move both in parallel. "
            "Output strictly in JSON conform to the TaskPlan schema."
        )
        
        contents = [system_prompt, f"User Command: {task_description}"]
        if self.latest_image:
            contents.append(self.latest_image)
            
        try:
            response = self.gemini_client.models.generate_content(
                model=self.model_name,
                contents=contents,
                config=types.GenerateContentConfig(
                    response_mime_type="application/json",
                    response_schema=TaskPlan,
                    temperature=0.1,
                ),
            )
            
            json_plan = response.text
            self.get_logger().info(f"Generated Plan: {json_plan}")
            
            result.success = True
            result.vlm_plan_json = json_plan
            result.message = "Bimanual plan generated successfully."
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
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
