import os
import time
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
        self.model_name = "gemini-2.0-flash" # Use flash for speed
        self.last_plan_cache = None
        self.cb_group = ReentrantCallbackGroup()
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10,
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
        task_description = goal_handle.request.task_description
        self.get_logger().info(f"Executing VLM planning goal for: '{task_description}'")
        result = VlmQuery.Result()
        
        # --- Safety Cache: Only use if task is identical ---
        if self.last_plan_cache is not None and getattr(self, "last_task_input", "") == task_description:
            self.get_logger().info("Using CACHED plan for matching task description.")
            result.success = True
            result.vlm_plan_json = self.last_plan_cache
            result.message = "Cached plan returned."
            goal_handle.succeed()
            return result

        self.last_task_input = task_description
        
        # GENERALIZED BIMANUAL PROMPT
        system_prompt = (
            "You are the Master Orchestrator for a dual-arm Franka Research 3 robot system. "
            "Your goal is to provide a SMOOTH, PARALLEL, and COORDINATED plan for any task. "
            "The system executes two arm lanes SIMULTANEOUSLY based on your JSON array. "
            "\n\n--- PERCEPTION ---\n"
            "You have access to the camera image to evaluate the scene. "
            "YOLO is ONLINE for precise 3D localization. "
            "You MUST call 'FIND_OBJECT' on an object before any arm can 'PICK' it. "
            "Predefined static frames you can use without finding: 'shared' (table relay point), 'box' (destination container).\n"
            "\n--- WORKSPACE TOPOLOGY ---\n"
            "- The robot operates a 'right_arm' and a 'left_arm'. "
            "- 'shared': Center of the table, reachable by BOTH arms. "
            "Observe the provided image to determine where objects are located. Assign 'PICK' actions to the arm that is physically closer to the object's zone.\n"
            "\n--- TABLE HANDOVER STRATEGY (If needed) ---\n"
            "If an object must be moved between zones reachable by different arms, use a 'shared' handover:\n"
            "1. Donor Arm: 'PICK' (from source) -> 'PLACE' (at 'shared').\n"
            "2. Recipient Arm: 'WAIT' (for donor) -> 'PICK' (from 'shared') -> 'PLACE' (at dest).\n"
            "\n--- PARALLELISMO SPINTO (Advanced Coordination) ---\n"
            "To maximize efficiency:\n"
            "- If an arm is waiting for the other (using 'WAIT'), always prepend it with a 'MOVE_HOME' action with 'target_pose_name': 'midway'.\n"
            "- This pre-positions the arm closer to its next objective while simultaneous work occurs elsewhere.\n"
            "\nOutput strictly in JSON conform to the TaskPlan schema without external text."
        )

        contents = [system_prompt, f"User Command: {task_description}"]
        if self.latest_image:
            contents.append(self.latest_image)
            
        max_retries = 3
        base_delay = 5.0
        
        for attempt in range(max_retries):
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
                
                self.last_plan_cache = json_plan
                result.success = True
                result.vlm_plan_json = json_plan
                result.message = "Bimanual plan generated successfully."
                goal_handle.succeed()
                return result

            except Exception as e:
                self.get_logger().error(f"Gemini Planning Failed (Attempt {attempt+1}/{max_retries}): {e}")
                if attempt < max_retries - 1:
                    time.sleep(base_delay)
                    base_delay *= 2
                else:
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
