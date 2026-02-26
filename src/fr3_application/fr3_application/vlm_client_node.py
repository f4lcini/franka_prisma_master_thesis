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

from pydantic import BaseModel, Field
from google import genai
from google.genai import types

# Define the structured output format for Gemini using Pydantic
class VlmDecision(BaseModel):
    # Putting reasoning FIRST forces Gemini to think before outputting the final decisions
    reasoning: str = Field(description="Step-by-step logical deduction of where the object is located in the image relative to the robotic arms, and which arm constraint applies. You MUST analyze reachability and singularities before picking an arm.")
    target_label: str = Field(description="The object the robot needs to interact with, e.g., 'red cube', 'apple', 'ball'. Use 'none' if not specified.")
    action_choice: str = Field(description="The primitive action to execute. Must be one of: 'pick', 'place', 'push', 'handover', 'none'")
    selected_arm: str = Field(description="The arm to use for the action. Must be one of: 'left', 'right', 'bimanual'")
    handover_height_z: float = Field(description="The estimated height in meters (relative to the table surface, Z=0.0) at which the robots should safely hand over the object to each other. For example, 0.3 for a safe mid-air pass above obstacles. If the task does not involve a handover, return 0.0.")

class VlmClientNode(Node):
    def __init__(self):
        super().__init__('vlm_client_node')
        
        self.get_logger().info("Initializing VLM Client Node...")
        
        # Init CvBridge for image conversion
        self.cv_bridge = CvBridge()
        self.latest_image = None
        
        # Setup Gemini Client
        api_key = os.environ.get("GEMINI_API_KEY")
        if not api_key:
            self.get_logger().error("GEMINI_API_KEY environment variable not set. Node will fail to process requests.")
        self.gemini_client = genai.Client(api_key=api_key)
        self.model_name = "gemini-2.5-flash" # Use the fast, multimodal model
        
        # We need a reentrant callback group so we can receive images while processing an action
        self.cb_group = ReentrantCallbackGroup()
        
        # Subscriber for the raw camera stream
        # TODO: Update the topic name to match your actual camera topic
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10,
            callback_group=self.cb_group
        )
        
        # Action Server
        self._action_server = ActionServer(
            self,
            VlmQuery,
            'vlm_query',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info("VLM Client Node Ready. Waiting for goals on /vlm_query action server.")

    def image_callback(self, msg: Image):
        """Cache the latest image from the camera."""
        try:
            # Convert ROS Image to OpenCV, then to PIL Image for Gemini
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
        """Process the goal using Gemini API."""
        self.get_logger().info('Executing VLM Query...')
        
        feedback_msg = VlmQuery.Feedback()
        result = VlmQuery.Result()
        
        task_description = goal_handle.request.task_description
        image_provided_in_goal = goal_handle.request.image_raw
        
        # 1. Image preparation
        pil_image = None
        if image_provided_in_goal.data:
            feedback_msg.current_state = "Processing image provided in request..."
            goal_handle.publish_feedback(feedback_msg)
            try:
                cv_img = self.cv_bridge.imgmsg_to_cv2(image_provided_in_goal, desired_encoding='rgb8')
                pil_image = PIL.Image.fromarray(cv_img)
            except Exception as e:
                self.get_logger().error(f"Failed to decode image in request: {e}")
                
        elif self.latest_image is not None:
             feedback_msg.current_state = "Using latest camera frame..."
             goal_handle.publish_feedback(feedback_msg)
             pil_image = self.latest_image
        else:
             self.get_logger().warn("No image provided in request, and no camera frames received. Proceeding with text only (not recommended).")

        # 2. Call Gemini API
        feedback_msg.current_state = "Sending request to Gemini..."
        goal_handle.publish_feedback(feedback_msg)
        
        system_prompt = (
            "You are the high-level cognitive planner for a dual-arm robot (Left Arm and Right Arm). "
            "You receive an image of your workspace and a user command. "
            "CRITICAL KINEMATIC RULES YOU MUST OBEY:\n"
            "1. WORKSPACE DIVISION: Objects physically located on the right side of the image MUST be picked by the 'right' arm. Objects on the left side of the image MUST be picked by the 'left' arm.\n"
            "2. SINGULARITIES: An arm CANNOT safely reach across the center line to the opposite side of the table due to joint singularities and reach limits.\n"
            "3. If an object (e.g., 'ball') is physically positioned on the right half of the table, YOU MUST select 'right' as the arm_selection, and absolutely NEVER select 'left' or 'any'.\n"
            "4. Be precise: evaluate the object's X coordinate in the image. If X is in the right half, choose 'right'. If X is in the left half, choose 'left'.\n"
            "5. HANDOVER DYNAMICS: If the user explicitly asks to pass an object between arms or implies a handover, you MUST evaluate a safe Z height (in meters) to perform the exchange, avoiding collisions with the table or objects below. Estimate this value (e.g., 0.25, 0.40) based on the scene context. If not applicable, output 0.0.\n\n"
            "Analyze the image, apply the kinematic rules step-by-step in the 'reasoning' field, and then output strictly the requested JSON structure."
        )
        
        contents = [system_prompt, f"User Command: {task_description}"]
        if pil_image:
            contents.append(pil_image)
            
        try:
            # Note: The new generate_content blocks until completion. 
            # In a ROS 2 async execute_callback, this might block the executor thread marginally, 
            # but since we use a MultiThreadedExecutor, it is acceptable.
            response = self.gemini_client.models.generate_content(
                model=self.model_name,
                contents=contents,
                config=types.GenerateContentConfig(
                    response_mime_type="application/json",
                    response_schema=VlmDecision,
                    temperature=0.1, # Low temperature for more deterministic JSON outputs
                ),
            )
            
            # 3. Parse and Return Results
            text_response = response.text
            self.get_logger().info(f"Gemini Raw Output: {text_response}")
            
            # The output is a JSON string matching the Pydantic schema
            # We use Pydantic to validate and parse it back into an object
            decision = VlmDecision.model_validate_json(text_response)
            
            result.success = True
            result.target_label = decision.target_label
            result.action_choice = decision.action_choice
            result.arm_selection = decision.selected_arm
            result.handover_height_z = decision.handover_height_z
            result.reasoning = decision.reasoning
            
            goal_handle.succeed()
            self.get_logger().info(f"VLM Query Succeeded: Action={result.action_choice}, Arm={result.arm_selection}, Handover Z={result.handover_height_z}m")
            return result

        except Exception as e:
            self.get_logger().error(f"Gemini API Call Failed: {e}")
            result.success = False
            result.reasoning = f"API Error: {str(e)}"
            goal_handle.abort()
            return result

def main(args=None):
    rclpy.init(args=args)
    node = VlmClientNode()
    
    # We use a MultiThreadedExecutor so the image callback can fire while execute_callback is waiting for Gemini
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
