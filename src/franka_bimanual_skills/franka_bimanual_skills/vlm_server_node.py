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
            "You are the task planner for a dual-arm Franka Research 3 robot system.\n"
            "Your ONLY job is to output a JSON plan following the EXACT structure shown below.\n\n"

            "--- MANDATORY OUTPUT STRUCTURE ---\n"
            "Each arm sequence MUST contain EXACTLY these 4 steps in this exact order:\n"
            "  1. FIND_OBJECT  (localize the object with YOLO)\n"
            "  2. PICK         (grasp the object)\n"
            "  3. PLACE        (deposit the object at its destination)\n"
            "  4. MOVE_HOME    (return arm to rest)\n\n"

            "FORBIDDEN: Do NOT add SYNC_BARRIER, WAIT, or any other step. Do NOT add more than 4 steps per arm.\n\n"

            "--- EXAMPLE OUTPUT (follow this structure exactly) ---\n"
            '{\n'
            '  "task_name": "Sort items",\n'
            '  "left_arm_sequence": [\n'
            '    {"action": "FIND_OBJECT", "target_name": "sports ball", "arm": "left_arm"},\n'
            '    {"action": "PICK",        "target_name": "sports ball", "arm": "left_arm"},\n'
            '    {"action": "PLACE",       "target_name": "box_ws_sx",   "arm": "left_arm"},\n'
            '    {"action": "MOVE_HOME",   "arm": "left_arm", "pose_name": "ready"}\n'
            '  ],\n'
            '  "right_arm_sequence": [\n'
            '    {"action": "FIND_OBJECT", "target_name": "sports ball", "arm": "right_arm"},\n'
            '    {"action": "PICK",        "target_name": "sports ball", "arm": "right_arm"},\n'
            '    {"action": "PLACE",       "target_name": "box_ws_dx",   "arm": "right_arm"},\n'
            '    {"action": "MOVE_HOME",   "arm": "right_arm", "pose_name": "ready"}\n'
            '  ]\n'
            '}\n\n'

            "--- RULES ---\n"
            "- left_arm handles objects on the LEFT side (X < 0). Destination: 'box_ws_sx'.\n"
            "- right_arm handles objects on the RIGHT side (X >= 0). Destination: 'box_ws_dx'.\n"
            "- Use ONLY the EXACT label string from YOLO detections (e.g. 'sports ball', 'bottle', 'cup').\n"
            "- If one side has no detected object, use MOVE_HOME as the only step for that arm.\n"
            "- Do NOT include 'chair', 'person', or any label not relevant to the sorting task.\n\n"

            "Format your output strictly as a JSON matching the TaskPlan schema with no markdown."
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
                "\nCRITICAL RULE: You MUST ONLY use these detected physical objects in your plan! "
                "Use the exact Label string. Assign each object to the arm indicated by its Spatial Side. "
                "Do NOT use generic names like 'left_item' or 'right_item'.\n\n"
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
                    
                    # ── Normalizza il piano alla struttura esatta FIND→PICK→PLACE→HOME ──
                    # Questo garantisce che il piano VLM sia identico a EXP1_sort_items.json
                    try:
                        plan_obj = json.loads(json_plan)
                        for arm_key in ['left_arm_sequence', 'right_arm_sequence']:
                            seq = plan_obj.get(arm_key, [])
                            # Prendi solo il primo passo di ogni tipo (ordine naturale)
                            find  = next((s for s in seq if s.get('action') == 'FIND_OBJECT'), None)
                            pick  = next((s for s in seq if s.get('action') == 'PICK'), None)
                            place = next((s for s in seq if s.get('action') == 'PLACE'), None)
                            home  = next((s for s in seq if s.get('action') == 'MOVE_HOME'), None)
                            normalized = [s for s in [find, pick, place, home] if s is not None]
                            if len(normalized) == 4:
                                plan_obj[arm_key] = normalized
                            # Se manca qualcosa, lascia invariata (caso degradato)
                        json_plan = json.dumps(plan_obj, indent=2)
                        self.get_logger().info(f"✅ Piano normalizzato a struttura FIND→PICK→PLACE→HOME")
                    except Exception as e:
                        self.get_logger().warn(f"⚠️ Normalizzazione piano fallita: {e} — uso output grezzo")

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
