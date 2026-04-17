import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from franka_custom_interfaces.action import MoveHome, PickObject, PlaceObject, GiveObject, TakeObject
from franka_custom_interfaces.srv import HandoverReady
from std_msgs.msg import Bool
import copy
import time
import traceback
import threading

from .config import PREDEFINED_TARGETS, WORLD_FRAME, READY_POSE_VALUES_LEFT, READY_POSE_VALUES_RIGHT, get_arm_config, apply_top_down_orientation, apply_donor_handover_orientation, apply_recipient_handover_orientation, parse_error_code

class SkillBehaviors:
    def __init__(self, node, robot_control_api, server_cb_group):
        self.node = node
        self.robot_control_api = robot_control_api
        self.logger = node.get_logger()

        # In-process rendezvous events (multi-stage handshake)
        self._donor_pre_pos_ready = threading.Event()
        self._recipient_pre_pos_ready = threading.Event()
        self._donor_ready = threading.Event()
        self._recipient_ready = threading.Event()
        self._recipient_grasped = threading.Event()

        self.home_server = ActionServer(
            self.node, MoveHome, 'move_home',
            execute_callback=self.execute_home, callback_group=server_cb_group)
        
        self.pick_server = ActionServer(
            self.node, PickObject, 'pick_object',
            execute_callback=self.execute_pick, callback_group=server_cb_group)
            
        self.place_server = ActionServer(
            self.node, PlaceObject, 'place_object',
            execute_callback=self.execute_place, callback_group=server_cb_group)

        self.give_server = ActionServer(
            self.node, GiveObject, 'give_object',
            execute_callback=self.execute_give, callback_group=server_cb_group)

        self.take_server = ActionServer(
            self.node, TakeObject, 'take_object',
            execute_callback=self.execute_take, callback_group=server_cb_group)

        # Publisher: fires when an object has been placed at 'shared' → wakes up the waiting arm
        self._handover_ready_pub = node.create_publisher(Bool, '/handover_ready', 10)
            
        self.logger.info("✅ Python Servers (Home, Pick, Place, Give, Take) Advertised!")



    async def execute_home(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"🔄 ---> Received MoveHome Action for: '{req_arm}' <---")
        result = MoveHome.Result()
        feedback = MoveHome.Feedback()
        
        arm_group, tcp_frame = get_arm_config(req_arm, self.logger)
        if not arm_group:
            self.logger.error("Aborting MoveHome: Invalid Arm Group.")
            goal_handle.abort()
            result.success = False
            result.message = f"Invalid arm group: '{req_arm}'"
            return result
        
        feedback.status = f"Planning MoveHome for {arm_group}"
        feedback.completion_percentage = 10.0
        goal_handle.publish_feedback(feedback)
            
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = arm_group
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 10
        
        if arm_group == "franka1_arm":
            ready_values = READY_POSE_VALUES_RIGHT
        else:
            ready_values = READY_POSE_VALUES_LEFT
            
        joint_names = [f"{arm_group.split('_')[0]}_fr3_joint{i+1}" for i in range(7)]
        
        c = Constraints()
        for name, val in zip(joint_names, ready_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
            
        req.goal_constraints.append(c)
        self.robot_control_api.apply_safety_limits(req)
        
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"
        
        goal_msg.request = req
        
        self.logger.info(f"📍 Requesting Joint Move (READY Pose) for {arm_group}...")
        feedback.status = f"Executing MoveHome for {arm_group}"
        feedback.completion_percentage = 50.0
        goal_handle.publish_feedback(feedback)
        
        try:
            handle = await self.robot_control_api.move_group_client.send_goal_async(goal_msg)
            if handle.accepted:
                self.logger.info("⏳ Goal accepted! Planning and moving...")
                res = await handle.get_result_async()
                error_val = res.result.error_code.val
                error_str = parse_error_code(error_val)
                success = (error_val == 1)
                if success:
                    self.logger.info(f"✅ MoveHome execution SUCCESS for {arm_group}!")
                    result.message = f"MoveHome completed for {arm_group}"
                else:
                    self.logger.error(f"❌ MoveHome FAILED: {error_str} ({error_val})")
                    result.message = f"MoveHome failed: {error_str} ({error_val})"
            else:
                self.logger.error("❌ MoveHome Goal REJECTED by MoveGroup server.")
                success = False
                result.message = "MoveHome goal rejected by MoveGroup"
        except Exception as e:
            self.logger.error(f"❌ Critical Exception during MoveHome: {e}")
            self.logger.error(traceback.format_exc())
            success = False
            result.message = f"Exception during MoveHome: {e}"

        result.success = success
        feedback.status = "Completed" if success else "Failed"
        feedback.completion_percentage = 100.0
        goal_handle.publish_feedback(feedback)
        
        if success:
            self.logger.info(f"👐 Opening gripper for {arm_group}...")
            await self.robot_control_api.send_gripper_goal(arm_group, width=0.08, max_effort=10.0)
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def execute_pick(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"📦 ---> Received Pick Object Action for: '{req_arm}' <---")
        result = PickObject.Result()
        feedback = PickObject.Feedback()
        
        # Load Parameters
        clearance = self.node.get_parameter('approach_clearance').value
        z_offset = self.node.get_parameter('pick_z_offset').value
        open_w = self.node.get_parameter('gripper_open_width').value
        grasp_w = self.node.get_parameter('gripper_grasp_width').value
        pause_s = self.node.get_parameter('safety_pause_short').value
        pause_l = self.node.get_parameter('safety_pause_long').value

        arm_group, tcp_frame = get_arm_config(req_arm, self.logger)
        if not arm_group:
            goal_handle.abort()
            result.success = False
            result.message = f"Invalid arm group: '{req_arm}'"
            return result
        
        req = goal_handle.request
        target_pose = req.target_pose.pose
        
        fid = req.target_pose.header.frame_id.lower()
        if fid in PREDEFINED_TARGETS:
            px, py, pz = PREDEFINED_TARGETS[fid]
            self.logger.info(f"📍 Overriding Pick with Predefined '{fid}': [{px}, {py}, {pz}]")
            target_pose.position.x = px
            target_pose.position.y = py
            target_pose.position.z = pz
        else:
            self.logger.info(f'🤖 Using Dynamic/YOLO Pose for Pick: [{target_pose.position.x:.3f}, {target_pose.position.y:.3f}]')

        apply_top_down_orientation(target_pose)
        
        grasp_z = target_pose.position.z + z_offset
        pre_grasp_z = grasp_z + clearance
        
        self.logger.info(f"🎯 Pick Target (Base Z={target_pose.position.z:.3f}, Final Z={grasp_z:.3f})")

        feedback.status = f"Pre-Step: Opening Gripper fully ({open_w}m)"
        goal_handle.publish_feedback(feedback)
        # Retry up to 3 times until gripper open is accepted
        for attempt in range(3):
            gripper_ok = await self.robot_control_api.send_gripper_goal(arm_group, width=open_w, max_effort=10.0)
            if gripper_ok:
                break
            self.logger.warning(f"Gripper open attempt {attempt+1} failed, retrying...")
            time.sleep(0.5)
        # Hard wait to let fingers physically reach the open position
        time.sleep(2.0)
        self.logger.info("✅ Gripper fully open — proceeding to approach.")

        pre_grasp = copy.deepcopy(req.target_pose)
        pre_grasp.pose.position.z = pre_grasp_z
        apply_top_down_orientation(pre_grasp.pose)
        
        self.logger.info(f"🚀 Approach to Z={pre_grasp_z:.3f}...")
        
        feedback.status = "Step 1/4: Moving to Pre-Grasp"
        feedback.completion_percentage = 25.0
        goal_handle.publish_feedback(feedback)
        if not await self.robot_control_api.send_pose_goal(arm_group, pre_grasp, tcp_frame):
            goal_handle.abort()
            result.success = False
            result.message = "Pre-Grasp move failed"
            return result
            
        feedback.status = f"Step 2/4: Vertical Descent to Z={grasp_z}"
        feedback.completion_percentage = 50.0
        goal_handle.publish_feedback(feedback)
        
        grasp_pose = copy.deepcopy(req.target_pose)
        grasp_pose.pose.position.z = grasp_z
        
        self.logger.info(f"📍 Executing Linear Descent (LIN) to Z={grasp_z}...")
        if not await self.robot_control_api.send_pose_goal_custom(arm_group, grasp_pose, tcp_frame, planner="LIN"):
            goal_handle.abort()
            result.success = False
            result.message = "Vertical descent (LIN) failed"
            return result

        time.sleep(pause_s)
            
        feedback.status = f"Step 3/4: Closing Gripper to {grasp_w}m"
        feedback.completion_percentage = 75.0
        goal_handle.publish_feedback(feedback)
        await self.robot_control_api.send_gripper_goal(arm_group, width=grasp_w, max_effort=30.0)

        time.sleep(pause_l) 
            
        feedback.status = "Step 4/4: Vertical Lift (Pilz LIN)"
        feedback.completion_percentage = 90.0
        goal_handle.publish_feedback(feedback)
        if not await self.robot_control_api.send_pose_goal_custom(arm_group, pre_grasp, tcp_frame, planner="LIN"):
            goal_handle.abort()
            result.success = False
            result.message = "Vertical lift (LIN) failed"
            return result
            
        time.sleep(pause_s)

        feedback.status = "Pick sequence completed"
        feedback.completion_percentage = 100.0
        goal_handle.publish_feedback(feedback)
        self.logger.info("🎉 Strategic Pick Sequence SUCCESS!")
        result.success = True
        result.message = f"Pick successful with {arm_group}"
        goal_handle.succeed()
        return result

    async def execute_place(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"📥 ---> Received Place Object Action for: '{req_arm}' <---")
        result = PlaceObject.Result()
        feedback = PlaceObject.Feedback()
        
        clearance = self.node.get_parameter('approach_clearance').value
        z_offset = self.node.get_parameter('place_z_offset').value
        open_w = self.node.get_parameter('gripper_open_width').value
        pause_s = self.node.get_parameter('safety_pause_short').value
        pause_l = self.node.get_parameter('safety_pause_long').value

        arm_group, tcp_frame = get_arm_config(req_arm, self.logger)
        if not arm_group:
            goal_handle.abort()
            result.success = False
            result.message = f"Invalid arm group: '{req_arm}'"
            return result
            
        req = goal_handle.request
        place_pose = req.place_pose.pose
        
        fid = req.place_pose.header.frame_id.lower()
        if fid in PREDEFINED_TARGETS:
            px, py, pz = PREDEFINED_TARGETS[fid]
            self.logger.info(f"📍 Overriding Place with Predefined '{fid}': [{px}, {py}, {pz}]")
            place_pose.position.x = px
            place_pose.position.y = py
            place_pose.position.z = pz
        else:
            self.logger.info(f'🤖 Using Dynamic/YOLO Pose for Pick: [{place_pose.position.x:.3f}, {place_pose.position.y:.3f}]')

        apply_top_down_orientation(place_pose)
        
        final_place_z = place_pose.position.z + z_offset
        pre_place_z = final_place_z + clearance
        
        self.logger.info(f"🎯 Place Target (Base Z={place_pose.position.z:.3f}, Final Z={final_place_z:.3f})")

        pre_place = copy.deepcopy(req.place_pose)
        pre_place.pose.position.z = pre_place_z
        apply_top_down_orientation(pre_place.pose)
        
        feedback.status = f"Step 1/4: Moving to Pre-Place (Z={pre_place_z:.3f})"
        goal_handle.publish_feedback(feedback)
        if not await self.robot_control_api.send_pose_goal(arm_group, pre_place, tcp_frame):
            goal_handle.abort()
            result.success = False
            result.message = "Pre-Place move failed"
            return result
            
        time.sleep(pause_s)
            
        feedback.status = f"Step 2/4: Vertical Descent to Z={final_place_z}"
        goal_handle.publish_feedback(feedback)
        
        place_target = copy.deepcopy(req.place_pose)
        place_target.pose.position.z = final_place_z
        
        self.logger.info(f"📍 Executing Linear Descent (LIN) to Z={final_place_z} for Place...")
        if not await self.robot_control_api.send_pose_goal_custom(arm_group, place_target, tcp_frame, planner="LIN"):
            goal_handle.abort()
            result.success = False
            result.message = "Vertical descent (LIN) failed for Place"
            return result
            
        time.sleep(pause_s)

        feedback.status = f"Step 3/4: Opening Gripper ({open_w}m)"
        goal_handle.publish_feedback(feedback)
        await self.robot_control_api.send_gripper_goal(arm_group, width=open_w, max_effort=10.0)
        
        time.sleep(pause_l)
        
        feedback.status = "Step 4/4: Vertical Retreat"
        goal_handle.publish_feedback(feedback)
        if not await self.robot_control_api.send_pose_goal_custom(arm_group, pre_place, tcp_frame, planner="LIN"):
             self.logger.warn("Retreat failed, but object placed.")
             
        time.sleep(pause_s)
             
        result.success = True
        result.message = f"Place successful with {arm_group}"
        self.logger.info("🎉 Strategic Place Sequence SUCCESS!")
        
        # If the object was placed at 'shared', signal the waiting arm
        if fid == 'shared':
            self.logger.info("📡 Publishing /handover_ready to wake up recipient arm.")
            msg = Bool()
            msg.data = True
            self._handover_ready_pub.publish(msg)
        
        goal_handle.succeed()
        return result

    async def execute_give(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"🎁 ---> [GIVE] Action for: '{req_arm}' <---")
        result = GiveObject.Result()
        
        arm_group, tcp_frame = get_arm_config(req_arm, self.logger)
        target_pose = goal_handle.request.handover_pose.pose
        
        fid = goal_handle.request.handover_pose.header.frame_id.lower()
        if fid in PREDEFINED_TARGETS:
            px, py, pz = PREDEFINED_TARGETS[fid]
            target_pose.position.x, target_pose.position.y, target_pose.position.z = px, py, pz
            goal_handle.request.handover_pose.header.frame_id = WORLD_FRAME
        else:
            self.logger.info(f'🤖 Using Dynamic Pose for Give: [{target_pose.position.x:.3f}, {target_pose.position.y:.3f}]')

        apply_donor_handover_orientation(target_pose)
        
        give_pose = copy.deepcopy(goal_handle.request.handover_pose)
        give_pose.header.frame_id = WORLD_FRAME
        give_pose.pose = target_pose

        # Donor approaches from ABOVE along Z axis
        donor_z_offset = self.node.get_parameter('handover_donor_z_offset').value
        pre_give_pose = copy.deepcopy(give_pose)
        pre_give_pose.pose.position.z += donor_z_offset
        
        self.logger.info(f"[{req_arm}] Moving to Pre-Handover Position (above, Z+{donor_z_offset})...")
        if not await self.robot_control_api.send_pose_goal(arm_group, pre_give_pose, tcp_frame, is_handover=True):
            goal_handle.abort()
            return result

        # --- HANDSHAKE STAGE 1: SIGNAL DONOR AT PRE-POSE ---
        self.logger.info(f"[{req_arm}] At Pre-Pose. Signaling DONOR_PRE_POS...")
        self._donor_pre_pos_ready.set()
        
        timeout = self.node.get_parameter('handover_timeout_sec').value
        self.logger.info(f"[{req_arm}] Waiting for RECIPIENT_PRE_POS...")
        if not self._recipient_pre_pos_ready.wait(timeout=timeout):
            self.logger.error("[Handshake] Timeout: recipient never reached pre-pose!")
            self._donor_pre_pos_ready.clear()
            goal_handle.abort()
            return result

        # --- HANDSHAKE STAGE 2: MOVE TO HANDOVER POINT ---
        self.logger.info(f"[{req_arm}] Both at Pre-Pose. Linear Approach (LIN) down to Handover Point...")
        if not await self.robot_control_api.send_pose_goal_custom(arm_group, give_pose, tcp_frame, planner="LIN", is_handover=True):
            self._donor_pre_pos_ready.clear()
            goal_handle.abort()
            return result

        # --- HANDSHAKE STAGE 3: SIGNAL DONOR READY ---
        self.logger.info(f"[{req_arm}] At handover point. Signaling DONOR_READY...")
        self._donor_ready.set()
        
        # --- HANDSHAKE STAGE 4: WAIT FOR RECIPIENT TO ARRIVE AT POINT ---
        self.logger.info(f"[{req_arm}] Waiting for RECIPIENT_READY...")
        if not self._recipient_ready.wait(timeout=timeout):
            self.logger.error("[Handshake] Timeout: recipient never reached handover point!")
            self._donor_ready.clear()
            self._donor_pre_pos_ready.clear()
            goal_handle.abort()
            return result

        # --- HANDSHAKE STAGE 5: RELEASE OBJECT ---
        self.logger.info(f"[{req_arm}] Recipient ready. Releasing object...")
        open_w = self.node.get_parameter('gripper_open_width').value
        await self.robot_control_api.send_gripper_goal(arm_group, width=open_w)
        
        # --- HANDSHAKE STAGE 6: WAIT FOR RECIPIENT TO GRASP ---
        self.logger.info(f"[{req_arm}] Object released. Waiting for RECIPIENT_GRASPED signal...")
        if not self._recipient_grasped.wait(timeout=timeout):
            self.logger.warning("[Handshake] Warning: recipient never signaled GRASPED. Retreating anyway.")

        self.logger.info(f"[{req_arm}] Starting Linear Retreat (LIN)...")
        await self.robot_control_api.send_pose_goal_custom(arm_group, pre_give_pose, tcp_frame, planner="LIN", is_handover=True)

        # Final cleanup for donor arm
        self._donor_ready.clear()
        self._donor_pre_pos_ready.clear()
        
        self.logger.info(f"✅ [GIVE] SUCCESS for {req_arm}")
        goal_handle.succeed()
        result.success = True
        return result

    async def execute_take(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"🤝 ---> [TAKE] Action for: '{req_arm}' <---")
        result = TakeObject.Result()
        
        arm_group, tcp_frame = get_arm_config(req_arm, self.logger)
        target_pose = goal_handle.request.handover_pose.pose
        
        fid = goal_handle.request.handover_pose.header.frame_id.lower()
        if fid in PREDEFINED_TARGETS:
            px, py, pz = PREDEFINED_TARGETS[fid]
            target_pose.position.x, target_pose.position.y, target_pose.position.z = px, py, pz
            goal_handle.request.handover_pose.header.frame_id = WORLD_FRAME
        else:
            self.logger.info(f'🤖 Using Dynamic Pose for Take: [{target_pose.position.x:.3f}, {target_pose.position.y:.3f}]')

        apply_recipient_handover_orientation(target_pose)
        
        take_pose = copy.deepcopy(goal_handle.request.handover_pose)
        take_pose.header.frame_id = WORLD_FRAME
        take_pose.pose = target_pose

        # Recipient approaches LATERALLY along X axis (avoids Z collision with donor)
        recipient_x_offset = self.node.get_parameter('handover_recipient_x_offset').value
        pre_take_pose = copy.deepcopy(take_pose)
        pre_take_pose.pose.position.x += recipient_x_offset  # offset in X, not Z

        # --- HANDSHAKE STAGE 1: MOVE TO PRE-POSE ---
        self.logger.info(f"[{req_arm}] Moving to Pre-Handover Position (lateral, X+{recipient_x_offset})...")
        if not await self.robot_control_api.send_pose_goal(arm_group, pre_take_pose, tcp_frame, is_handover=True):
            goal_handle.abort()
            return result
        
        # --- HANDSHAKE STAGE 2: SIGNAL RECIPIENT AT PRE-POSE ---
        self.logger.info(f"[{req_arm}] At Pre-Pose. Signaling RECIPIENT_PRE_POS...")
        self._recipient_pre_pos_ready.set()
        
        timeout = self.node.get_parameter('handover_timeout_sec').value
        self.logger.info(f"[{req_arm}] Waiting for DONOR_PRE_POS...")
        if not self._donor_pre_pos_ready.wait(timeout=timeout):
            self.logger.error("[Handshake] Timeout: donor never reached pre-pose!")
            self._recipient_pre_pos_ready.clear()
            goal_handle.abort()
            return result

        # --- HANDSHAKE STAGE 3: WAIT FOR DONOR TO BE AT HANDOVER POINT ---
        # We wait for the donor to be at the final point BEFORE we approach to avoid collisions
        self.logger.info(f"[{req_arm}] Waiting for DONOR_READY (at handover point)...")
        if not self._donor_ready.wait(timeout=timeout):
             self.logger.error("[Handshake] Timeout: donor never reached handover point!")
             self._recipient_pre_pos_ready.clear()
             goal_handle.abort()
             return result

        # --- HANDSHAKE STAGE 4: MOVE TO HANDOVER POINT ---
        self.logger.info(f"[{req_arm}] Starting Linear Approach (LIN) along X to Handover Point...")
        if not await self.robot_control_api.send_pose_goal_custom(arm_group, take_pose, tcp_frame, planner="LIN", is_handover=True):
            self.logger.error(f"[{req_arm}] Linear Approach FAILED!")
            self._recipient_pre_pos_ready.clear()
            goal_handle.abort()
            return result

        # --- HANDSHAKE STAGE 5: SIGNAL RECIPIENT READY AND GRASP ---
        self.logger.info(f"[{req_arm}] At point. Signaling RECIPIENT_READY and grasping...")
        self._recipient_ready.set()
        
        grasp_w = self.node.get_parameter('gripper_grasp_width').value
        await self.robot_control_api.send_gripper_goal(arm_group, width=grasp_w)
        
        self.logger.info(f"[{req_arm}] Grasp complete. Signaling RECIPIENT_GRASPED...")
        self._recipient_grasped.set()

        # ─── STEP 6: Retreat ───
        pause_s = self.node.get_parameter('safety_pause_short').value
        time.sleep(pause_s)
        
        self.logger.info(f"[{req_arm}] Starting Linear Retreat (LIN) along X...")
        await self.robot_control_api.send_pose_goal_custom(arm_group, pre_take_pose, tcp_frame, planner="LIN", is_handover=True)

        # Final cleanup for recipient arm
        self._recipient_ready.clear()
        self._recipient_grasped.clear()
        self._recipient_pre_pos_ready.clear()
        
        self.logger.info(f"✅ [TAKE] SUCCESS for {req_arm}")
        goal_handle.succeed()
        result.success = True
        return result
