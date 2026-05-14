import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from franka_custom_interfaces.action import MoveHome, PickObject, PlaceObject, GiveObject, TakeObject, ParallelMove
from franka_custom_interfaces.srv import HandoverReady
from std_msgs.msg import Bool
import copy
import time
import traceback
import threading
from std_srvs.srv import Trigger

from .config import (
    PREDEFINED_TARGETS, WORLD_FRAME, TARGET_OFFSETS,
    READY_POSE_VALUES_LEFT, READY_POSE_VALUES_RIGHT, 
    MIDWAY_POSE_VALUES_LEFT, MIDWAY_POSE_VALUES_RIGHT, 
    OFFSET_POSE_VALUES_LEFT, OFFSET_POSE_VALUES_RIGHT, 
    get_arm_config, apply_top_down_orientation, apply_rotated_top_down_orientation,
    apply_donor_handover_orientation, apply_recipient_handover_orientation, 
    parse_error_code
)

class SkillBehaviors:
    def __init__(self, node, robot_control_api, server_cb_group):
        self.node = node
        self.robot_control_api = robot_control_api
        self.logger = node.get_logger()
        # --- MUTEX CLIENTS ---
        self._mutex_acquire_client = self.node.create_client(Trigger, '/acquire_shared_zone', callback_group=server_cb_group)
        self._mutex_release_client = self.node.create_client(Trigger, '/release_shared_zone', callback_group=server_cb_group)

        self.cb_groups = {
            'home': rclpy.callback_groups.ReentrantCallbackGroup(),
            'pick': rclpy.callback_groups.ReentrantCallbackGroup(),
            'place': rclpy.callback_groups.ReentrantCallbackGroup()
        }

        self.home_server = ActionServer(
            self.node, MoveHome, 'move_home',
            execute_callback=self.execute_home, callback_group=self.cb_groups['home'])
        
        self.pick_server = ActionServer(
            self.node, PickObject, 'pick_object',
            execute_callback=self.execute_pick, callback_group=self.cb_groups['pick'])
            
        self.place_server = ActionServer(
            self.node, PlaceObject, 'place_object',
            execute_callback=self.execute_place, callback_group=self.cb_groups['place'])

        # Publisher: fires when an object has been placed at 'shared'
        self._handover_ready_pub = node.create_publisher(Bool, '/handover_ready', 10)
            
        self.logger.info("✅ Python Servers (Home, Pick, Place, Give, Take) Advertised!")
        
    def _get_offset(self, target_id, param_name):
        """Get target-specific offset if available, otherwise fallback to default param."""
        if target_id and target_id in TARGET_OFFSETS:
            if param_name in TARGET_OFFSETS[target_id]:
                val = TARGET_OFFSETS[target_id][param_name]
                self.logger.info(f"💡 Using target-specific override for {target_id}: {param_name}={val}")
                return val
        return self.node.get_parameter(param_name).value

    async def _is_zone_critical(self, arm_group, target_pose, fid):
        """Determina se la posizione target richiede l'uso del Mutex."""
        x = target_pose.position.x
        
        # 1. Se il target è esplicitamente 'shared' o 'box' (per il destro)
        if fid in ["shared", "box", "target_object"]:
            # Se il destro va nella box (lato sinistro) è critico
            if "franka1" in arm_group and x < 0.1: return True
            # Se il sinistro va nel target_object (lato destro) è critico
            if "franka2" in arm_group and x > -0.1: return True
            # La zona shared è sempre critica
            if fid == "shared": return True

        # 2. Controllo basato sulle coordinate X (Zona centrale di sicurezza)
        # Se un braccio si avvicina a meno di 15cm dal centro del tavolo, chiede il lock
        if abs(x) < 0.15:
            return True
            
        return False

    async def _request_mutex(self, arm_group):
        """Richiede l'accesso esclusivo alla zona condivisa."""
        if not self._mutex_acquire_client.wait_for_server(timeout_sec=1.0):
            self.logger.warning(f"[{arm_group}] Mutex server non trovato, procedo senza lock!")
            return False
        self.logger.info(f"🛡️ [{arm_group}] Richiesta accesso zona condivisa...")
        await self._mutex_acquire_client.call_async(Trigger.Request())
        self.logger.info(f"✅ [{arm_group}] Accesso GARANTITO.")
        return True

    async def _release_mutex(self, arm_group):
        """Rilascia la zona condivisa."""
        if not self._mutex_release_client.wait_for_server(timeout_sec=1.0): return
        self.logger.info(f"🔓 [{arm_group}] Rilascio zona condivisa.")
        await self._mutex_release_client.call_async(Trigger.Request())


    def safe_publish_feedback(self, goal_handle, feedback):
        try:
            if goal_handle.is_active:
                goal_handle.publish_feedback(feedback)
        except Exception:
            pass

    def safe_abort(self, goal_handle):
        try:
            if goal_handle.is_active:
                goal_handle.abort()
        except Exception:
            pass

    def safe_succeed(self, goal_handle):
        try:
            if goal_handle.is_active:
                goal_handle.succeed()
        except Exception:
            pass

    async def execute_home(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"🔄 Received MoveHome for: '{req_arm}'")
        result = MoveHome.Result()
        
        arm_group, _ = get_arm_config(req_arm, self.logger)
        pose_target = goal_handle.request.pose_name.lower() or "ready"
        
        if arm_group == "franka1_arm":
            ready_values = MIDWAY_POSE_VALUES_RIGHT if pose_target == "midway" else \
                           OFFSET_POSE_VALUES_RIGHT if pose_target == "offset_ready" else \
                           READY_POSE_VALUES_RIGHT
        else:
            ready_values = MIDWAY_POSE_VALUES_LEFT if pose_target == "midway" else \
                           OFFSET_POSE_VALUES_LEFT if pose_target == "offset_ready" else \
                           READY_POSE_VALUES_LEFT
            
        success = await self.robot_control_api.send_moveit_goal_async(arm_group, joint_target=ready_values, planner="PTP")

        result.success = success
        if success:
            open_w = self.node.get_parameter('gripper_open_width').value
            await self.robot_control_api.send_gripper_goal_async(arm_group, width=open_w)
            self.safe_succeed(goal_handle)
        else:
            self.safe_abort(goal_handle)
        return result

    async def execute_pick(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"📥 [DEBUG] execute_pick RICEVUTO per {req_arm}")
        result = PickObject.Result()
        
        arm_group, _ = get_arm_config(req_arm, self.logger)
        self.logger.info(f"🔍 [DEBUG] Arm group: {arm_group}")
        if not arm_group:
            self.safe_abort(goal_handle); result.success = False; return result
        
        req = goal_handle.request
        target_pose = req.target_pose.pose
        fid = req.target_pose.header.frame_id.lower()
        
        clearance = self._get_offset(fid, 'approach_clearance')
        z_offset = self._get_offset(fid, 'pick_z_offset')
        grasp_w = self._get_offset(fid, 'gripper_grasp_width') # <-- Corretto!
        open_w = self.node.get_parameter('gripper_open_width').value
        
        self.logger.info(f"🔍 DEBUG PICK: fid='{fid}', grasp_w={grasp_w}")
        
        if fid in PREDEFINED_TARGETS:
            px, py, pz = PREDEFINED_TARGETS[fid]
            target_pose.position.x, target_pose.position.y, target_pose.position.z = float(px), float(py), float(pz)
            req.target_pose.header.frame_id = WORLD_FRAME
        else:
            # Per oggetti YOLO, fid è il nome dell'oggetto. 
            # Ora che abbiamo preso gli offset, riportiamo il frame a 'table'
            req.target_pose.header.frame_id = WORLD_FRAME
        
        if "sports" in fid:
            apply_rotated_top_down_orientation(target_pose)
            self.logger.info(f"[{arm_group}] Using ROTATED (270deg) orientation for {fid}")
        else:
            apply_top_down_orientation(target_pose)
        
        # Apply X, Y offsets if specified
        x_offset = self._get_offset(fid, 'pick_x_offset')
        target_pose.position.x += x_offset
        # Y-offset is now handled by the localization node directly for sports ball
        
        grasp_z = target_pose.position.z + z_offset
        pre_grasp_z = grasp_z + clearance
        
        # 1. Open Gripper
        self.logger.info(f"[{arm_group}] Step 0: Opening Gripper")
        await self.robot_control_api.send_gripper_goal_async(arm_group, width=open_w)

        # --- MUTEX MANAGEMENT ---
        has_mutex = False
        self.logger.info(f"🔍 [DEBUG] Controllo criticità zona per {fid} a X={target_pose.position.x:.3f}...")
        is_critical = await self._is_zone_critical(arm_group, target_pose, fid)
        self.logger.info(f"🔍 [DEBUG] Zona critica? {is_critical}")
        if is_critical:
            has_mutex = await self._request_mutex(arm_group)

        # 2. Approach (PTP)
        self.logger.info(f"[{arm_group}] Step 1: Approach (PTP)")
        pre_grasp = copy.deepcopy(req.target_pose)
        pre_grasp.pose.position.z = pre_grasp_z
        if not await self.robot_control_api.send_moveit_goal_async(arm_group, target_pose=pre_grasp, planner="PTP"):
            self.logger.error(f"[{arm_group}] Step 1 (Approach) FAILED")
            self.safe_abort(goal_handle); result.success = False; return result
            
        # 3. Descent (PTP)
        self.logger.info(f"[{arm_group}] Step 2: Descent (PTP)")
        grasp_pose = copy.deepcopy(req.target_pose)
        grasp_pose.pose.position.z = grasp_z
        if not await self.robot_control_api.send_moveit_goal_async(arm_group, target_pose=grasp_pose, planner="PTP"):
            self.logger.error(f"[{arm_group}] Step 2 (Descent) FAILED")
            self.safe_abort(goal_handle); return result

        # 4. Grasp
        self.logger.info(f"[{arm_group}] Step 3: Grasping")
        await self.robot_control_api.send_gripper_goal_async(arm_group, width=grasp_w)
        
        # 5. Lift (PTP)
        self.logger.info(f"[{arm_group}] Step 4: Lift (PTP)")
        await self.robot_control_api.send_moveit_goal_async(arm_group, target_pose=pre_grasp, planner="PTP")
        
        # --- RELEASE MUTEX ---
        if has_mutex:
            await self._release_mutex(arm_group)
            
        result.success = True
        self.safe_succeed(goal_handle)
        return result

    async def execute_place(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"📥 ---> Received Place Object Action for: '{req_arm}' <---")
        result = PlaceObject.Result()
        
        arm_group, _ = get_arm_config(req_arm, self.logger)
        req = goal_handle.request
        place_pose = req.place_pose.pose
        fid = req.place_pose.header.frame_id.lower()

        if fid in PREDEFINED_TARGETS:
            px, py, pz = PREDEFINED_TARGETS[fid]
            place_pose.position.x, place_pose.position.y, place_pose.position.z = float(px), float(py), float(pz)
            req.place_pose.header.frame_id = WORLD_FRAME
        else:
            req.place_pose.header.frame_id = WORLD_FRAME

        clearance = self._get_offset(fid, 'approach_clearance')
        z_offset = self._get_offset(fid, 'place_z_offset')
        open_w = self.node.get_parameter('gripper_open_width').value
        
        apply_top_down_orientation(place_pose)

        # Apply X, Y offsets if specified (using pick offsets as general object offsets for now)
        x_offset = self._get_offset(fid, 'pick_x_offset')
        y_offset = self._get_offset(fid, 'pick_y_offset')
        place_pose.position.x += x_offset
        place_pose.position.y += y_offset

        final_place_z = place_pose.position.z + z_offset
        pre_place_z = final_place_z + clearance
        
        pre_place = copy.deepcopy(req.place_pose)
        pre_place.pose.position.z = pre_place_z

        # --- MUTEX MANAGEMENT ---
        has_mutex = False
        self.logger.info(f"🔍 [DEBUG] Controllo criticità zona per PLACE '{fid}'...")
        is_critical = await self._is_zone_critical(arm_group, pre_place.pose, fid)
        self.logger.info(f"🔍 [DEBUG] Zona critica? {is_critical}")
        if is_critical:
            has_mutex = await self._request_mutex(arm_group)
        
        # 1. Approach
        self.logger.info(f"[{arm_group}] Step 1: Approach (PTP)")
        if not await self.robot_control_api.send_moveit_goal_async(arm_group, target_pose=pre_place, planner="PTP"):
            self.logger.error(f"[{arm_group}] Step 1 (Approach) FAILED")
            self.safe_abort(goal_handle); return result
            
        # 2. Descent
        self.logger.info(f"[{arm_group}] Step 2: Descent (LIN)")
        place_target = copy.deepcopy(req.place_pose)
        place_target.pose.position.z = final_place_z
        if not await self.robot_control_api.send_moveit_goal_async(arm_group, target_pose=place_target, planner="PTP"):
            self.logger.error(f"[{arm_group}] Step 2 (Descent) FAILED")
            self.safe_abort(goal_handle); return result
            
        # 3. Release
        self.logger.info(f"[{arm_group}] Step 3: Releasing")
        await self.robot_control_api.send_gripper_goal_async(arm_group, width=open_w)
        
        # Handover signal
        if fid == 'shared':
            self._handover_ready_pub.publish(Bool(data=True))
        
        # 4. Retreat
        self.logger.info(f"[{arm_group}] Step 4: Retreat (LIN)")
        await self.robot_control_api.send_moveit_goal_async(arm_group, target_pose=pre_place, planner="PTP")

        # --- RELEASE MUTEX ---
        if has_mutex:
            await self._release_mutex(arm_group)
             
        result.success = True
        self.safe_succeed(goal_handle)
        return result
