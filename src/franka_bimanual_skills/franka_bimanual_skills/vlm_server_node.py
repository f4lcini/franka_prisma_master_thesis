import os
import time
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from franka_custom_interfaces.action import VlmQuery
from franka_custom_interfaces.srv import ScanTable

from cv_bridge import CvBridge
import PIL.Image

from franka_bimanual_skills.skills_repertoire import TaskPlan
from google import genai
from google.genai import types


class VlmServerNode(Node):
    def __init__(self):
        super().__init__('vlm_server_node')
        self.get_logger().info("Initializing Generalized VLM Bimanual Orchestrator...")

        self.cv_bridge   = CvBridge()
        self.latest_image = None

        # ── Gemini API ────────────────────────────────────────────────────────
        api_key = os.environ.get("GEMINI_API_KEY")
        if not api_key:
            # Fallback: cerca il file .env sia in Docker (/mm_ws) che su host
            possible_paths = [
                "/mm_ws/KEY/.env",
                "/mm_ws/src/franka_bimanual_skills/launch/.env",
                "/home/hargalaten/Documents/vfalcini/franka_prisma_master_thesis/KEY/.env",
                os.path.join(os.getcwd(), ".env"),
            ]
            for p in possible_paths:
                if os.path.exists(p):
                    self.get_logger().info(f"🔑 Caricamento chiave API da {p}...")
                    with open(p, 'r') as f:
                        for line in f:
                            if '=' in line and not line.strip().startswith('#'):
                                k, v = line.strip().split('=', 1)
                                if k.strip() == "GEMINI_API_KEY":
                                    api_key = v.strip().strip('"').strip("'")
                                    os.environ["GEMINI_API_KEY"] = api_key
                                    break
                    break

        if not api_key:
            self.get_logger().error(
                "❌ GEMINI_API_KEY non definita e nessun file .env trovato!"
            )

        self.gemini_client = genai.Client(api_key=api_key)
        # Modelli in ordine di preferenza: se uno è a quota, si prova il successivo
        self.model_candidates = [
            "gemini-2.5-flash",
            "gemini-2.0-flash",
            "gemini-1.5-pro",
            "gemini-1.5-flash",
            "gemini-1.5-flash-8b"
        ]
        self.last_plan_cache = None
        self.last_task_input = ""

        self.cb_group = ReentrantCallbackGroup()

        # ── Camera image subscription (per passare l'immagine a Gemini) ──────
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        image_topic = self.get_parameter('image_topic').value
        self.get_logger().info(f"📹 VLM subscribing to: '{image_topic}'")
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10,
            callback_group=self.cb_group
        )

        # ── /scan_table service client (delega la percezione a object_localization_node) ──
        self.scan_table_client = self.create_client(
            ScanTable, 'scan_table',
            callback_group=self.cb_group
        )

        # ── VLM Action Server ─────────────────────────────────────────────────
        self._action_server = ActionServer(
            self, VlmQuery, 'vlm_query',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("VLM Bimanual Orchestrator Ready.")

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

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

    # ─────────────────────────────────────────────────────────────────────────
    # Scene scan via /scan_table service
    # ─────────────────────────────────────────────────────────────────────────

    def _call_scan_table(self):
        """
        Chiama il service /scan_table di object_localization_node e restituisce
        la lista di oggetti rilevati. Se il service non è disponibile, ritorna [].
        """
        if not self.scan_table_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "⚠️  /scan_table service non disponibile (object_localization_node attivo?). "
                "Il VLM pianificherà senza grounding visivo."
            )
            return []

        future = self.scan_table_client.call_async(ScanTable.Request())

        # Spin sincrono dentro l'executor multi-thread (safe con ReentrantCallbackGroup)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.done() or future.result() is None:
            self.get_logger().error("❌ /scan_table timeout o errore.")
            return []

        resp = future.result()
        if not resp.success:
            self.get_logger().warn(f"⚠️  /scan_table: {resp.message}")
            return []

        try:
            objects = json.loads(resp.scene_json)
            self.get_logger().info(f"✅ /scan_table: {len(objects)} oggetto/i rilevato/i")
            return objects
        except Exception as e:
            self.get_logger().error(f"❌ Errore parsing scene_json: {e}")
            return []

    # ─────────────────────────────────────────────────────────────────────────
    # Main VLM planning callback
    # ─────────────────────────────────────────────────────────────────────────

    async def execute_callback(self, goal_handle):
        task_description = goal_handle.request.task_description
        self.get_logger().info(f"Executing VLM planning goal for: '{task_description}'")
        result = VlmQuery.Result()

        # Safety cache: restituisce il piano precedente se il task è identico
        if self.last_plan_cache and self.last_task_input == task_description:
            self.get_logger().info("Using CACHED plan for matching task description.")
            result.success      = True
            result.vlm_plan_json = self.last_plan_cache
            result.message      = "Cached plan returned."
            goal_handle.succeed()
            return result

        self.last_task_input = task_description

        # ── System Prompt ────────────────────────────────────────────────────
        system_prompt = (
            "You are the core intelligence of the Semantic-Based Skill Orchestration Framework for Coordinating Bimanual Robotic Tasks (using two Franka Research 3 arms).\n"
            "Your objective is to act as a Vision-Language Model (VLM). You MUST directly analyze the provided image of the workspace to infer spatial semantics, object affordances, and potential collision zones before generating a JSON plan.\n"
            "CRITICAL: The generated JSON plan will be fed into a dynamic Behavior Tree engine. The 'left_arm_sequence' and 'right_arm_sequence' will be executed IN PARALLEL. You must use SYNC_BARRIER to explicitly synchronize them when sequential steps are required.\n\n"
            
            "--- INITIAL STATE ---\n"
            "Assume both arms are empty and currently in the 'ready' pose.\n\n"
            
            "--- AVAILABLE ACTIONS ---\n"
            "- FIND_OBJECT: Localize an object with YOLO. Requires 'target_name' and 'arm'.\n"
            "- PICK: Grasp an object. Requires 'target_name' and 'arm'. If picking from the shared zone, use 'target_name': 'shared' (NO FIND_OBJECT needed).\n"
            "- PLACE: Deposit an object. Requires 'target_name' ('box_ws_sx', 'box_ws_dx', or 'shared') and 'arm'.\n"
            "- MOVE_HOME: Move arm to a resting pose. Optionally requires 'pose_name' ('ready' or 'midway'). IMPORTANT: In 'midway' pose, the gripper DOES NOT open (useful for holding objects during coordination). The mission MUST always end with a MOVE_HOME (pose_name 'ready') for both arms.\n"
            "- SYNC_BARRIER: A synchronization point. The arm pauses until the other arm reaches its SYNC_BARRIER. Ensure both arms have the EXACT same number of SYNC_BARRIERs in their sequences.\n\n"
            
            "--- PREDEFINED LOCATIONS & POSES ---\n"
            "- 'box_ws_sx': The drop-off box located in the left workspace (accessible only by left_arm).\n"
            "- 'box_ws_dx': The drop-off box located in the right workspace (accessible only by right_arm).\n"
            "- 'shared': The common overlapping workspace in the center. Used to transfer objects between arms or place items centrally.\n"
            "- 'ready': The default safe resting pose for the arms.\n"
            "- 'midway': An intermediate safe pose where the gripper remains CLOSED (holding the object). Used while waiting for the other arm.\n\n"
            
            "--- SPATIAL & SAFETY RULES ---\n"
            "1. 'left_arm' operates on the left. It drops objects in 'box_ws_sx' or 'shared'.\n"
            "2. 'right_arm' operates on the right. It drops objects in 'box_ws_dx' or 'shared'.\n"
            "3. END OF MISSION: Both arms MUST ALWAYS finish their sequences with a 'MOVE_HOME' (pose_name 'ready').\n"
            "4. COLLISION AVOIDANCE & MAX PARALLELISM: Both arms CANNOT access 'shared' at the same time. To maximize parallel execution, delay the SYNC_BARRIER as much as possible. Place the SYNC_BARRIER immediately BEFORE the 'PLACE' action in 'shared', so both arms can FIND and PICK simultaneously without waiting.\n"
            "5. HANDOVERS: To transfer an object, the donor places it in 'shared' and waits (SYNC_BARRIER). The recipient waits (SYNC_BARRIER) until the donor is clear, then picks from 'shared'.\n"
            "6. INANIMATE OBJECTS ONLY: You MUST strictly ignore any detected object labeled 'person'. The robot can only physically manipulate inanimate items (e.g. 'sports ball', 'bottle', 'cup'). Do NEVER attempt to FIND or PICK a 'person'.\n\n"
            
            "--- TASK CONTEXTS (Reference Experiments) ---\n"
            "The system handles 3 physical setups. Infer the correct goal and plan accordingly:\n"
            "1. EXP1 (Sort Items): Both drop-off boxes are present. Goal: sort items independently into respective boxes.\n"
            "2. EXP2 (Transfer/Handover): Only the right box ('box_ws_dx') is present. Goal: items from the left must be transferred to the right arm via 'shared' to reach the box.\n"
            "3. EXP3 (Clear Workspace): The destination box is located centrally in the 'shared' zone. Goal: both arms must place all objects into 'shared' without colliding.\n\n"
            
            "--- OUTPUT FORMAT ---\n"
            "You must output STRICTLY a JSON object mapping to the TaskPlan schema. Here is a generic EXAMPLE (completely different from the actual experiments) to show the syntax:\n"
            "{\n"
            '  "left_arm_sequence": [\n'
            '    {"action": "MOVE_HOME", "pose_name": "midway", "arm": "left_arm"},\n'
            '    {"action": "MOVE_HOME", "pose_name": "ready", "arm": "left_arm"}\n'
            '  ],\n'
            '  "right_arm_sequence": [\n'
            '    {"action": "MOVE_HOME", "pose_name": "midway", "arm": "right_arm"},\n'
            '    {"action": "MOVE_HOME", "pose_name": "ready", "arm": "right_arm"}\n'
            '  ]\n'
            "}\n"
        )

        # ── YOLO Grounding via /scan_table ───────────────────────────────────
        detected_objects = self._call_scan_table()

        yolo_info = "--- CURRENT ACTIVE DETECTIONS FROM YOLOv26m (via /scan_table) ---\n"
        if detected_objects:
            yolo_info += "The YOLO vision system has currently detected the following physical objects on the table:\n"
            for idx, obj in enumerate(detected_objects):
                arm_hint = "left_arm" if obj.get('side') == "left_side" else "right_arm"
                yolo_info += (
                    f"- Object {idx+1}: Label = '{obj['label']}', "
                    f"Spatial Side = {obj.get('side','?')} → recommended arm: {arm_hint} "
                    f"(X={obj.get('x_world', 0.0):.3f}m, Confidence: {obj.get('conf', 0.0):.2f})\n"
                )
            yolo_info += (
                "\nCRITICAL RULE: While YOLO provides these base textual detections, you MUST analyze the attached image to verify spatial relationships, object accessibility, and plan safe bimanual sequences. "
                "Use the exact Label string from YOLO, but use your visual understanding to dictate the interaction order and SYNC_BARRIER placements. Do NOT use generic names like 'left_item' or 'right_item'.\n\n"
            )
        else:
            yolo_info += (
                "The YOLO vision system currently detects NO objects on the table.\n"
                "If the user requested sorting or picking, report that the table is empty.\n\n"
            )

        self.get_logger().info(f"👁️ YOLO TABLE SCAN RESULT:\n{yolo_info}")

        # ── Build Gemini request ──────────────────────────────────────────────
        full_system_prompt = system_prompt + "\n" + yolo_info
        contents = [full_system_prompt, f"User Command: {task_description}"]
        if self.latest_image:
            self.get_logger().info("📸 Camera image included in VLM query.")
            contents.append(self.latest_image)
        else:
            self.get_logger().warn("⚠️ No camera image — VLM will plan from text only.")

        # ── Call Gemini with model fallback on 429 ────────────────────────────
        for model_name in self.model_candidates:
            self.get_logger().info(f"🤖 Tentativo con modello: '{model_name}'...")
            max_retries = 5 if model_name == "gemini-2.5-flash" else 2
            base_delay  = 5.0

            for attempt in range(max_retries):
                try:
                    response = self.gemini_client.models.generate_content(
                        model=model_name,
                        contents=contents,
                        config=types.GenerateContentConfig(
                            response_mime_type="application/json",
                            response_schema=TaskPlan,
                            temperature=0.1,
                        ),
                    )
                    json_plan = response.text
                    
                    try:
                        plan_dict = json.loads(json_plan)
                        plan_dict["scene_inventory"] = detected_objects
                        plan_dict["vlm_input_prompt"] = full_system_prompt
                        json_plan = json.dumps(plan_dict, indent=2)
                    except Exception as e:
                        self.get_logger().error(f"Failed to inject scene_inventory into VLM plan: {e}")
                    
                    self.get_logger().info(f"✅ Piano generato con '{model_name}': {json_plan}")

                    self.last_plan_cache = json_plan
                    result.success       = True
                    result.vlm_plan_json  = json_plan
                    result.message       = f"Plan generated successfully with {model_name}."
                    goal_handle.succeed()
                    return result

                except Exception as e:
                    err_str  = str(e)
                    is_quota = "429" in err_str or "RESOURCE_EXHAUSTED" in err_str
                    is_unavailable = "503" in err_str or "UNAVAILABLE" in err_str
                    self.get_logger().error(
                        f"❌ '{model_name}' attempt {attempt+1}/{max_retries}: {e}"
                    )
                    if is_quota:
                        self.get_logger().warn(
                            f"⚠️  Quota 429 su '{model_name}', provo modello successivo..."
                        )
                        break
                    elif is_unavailable and attempt < max_retries - 1:
                        wait = base_delay * (attempt + 1)
                        self.get_logger().warn(
                            f"⚠️  503 su '{model_name}' (attempt {attempt+1}/{max_retries}), riprovo tra {wait:.0f}s..."
                        )
                        time.sleep(wait)
                    elif attempt < max_retries - 1:
                        time.sleep(base_delay)
                        base_delay *= 2
                    else:
                        break

        # Tutti i modelli hanno fallito
        result.success = False
        result.message = "Tutti i modelli Gemini hanno fallito (quota esaurita o errore API)."
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
