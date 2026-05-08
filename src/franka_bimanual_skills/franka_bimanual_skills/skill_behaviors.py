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

from .config import (
    PREDEFINED_TARGETS, WORLD_FRAME, TARGET_OFFSETS,
    READY_POSE_VALUES_LEFT, READY_POSE_VALUES_RIGHT, 
    MIDWAY_POSE_VALUES_LEFT, MIDWAY_POSE_VALUES_RIGHT, 
    OFFSET_POSE_VALUES_LEFT, OFFSET_POSE_VALUES_RIGHT, 
    get_arm_config, apply_top_down_orientation, 
    apply_donor_handover_orientation, apply_recipient_handover_orientation, 
    parse_error_code
)

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

        self.cb_groups = {
            'home': rclpy.callback_groups.ReentrantCallbackGroup(),
            'pick': rclpy.callback_groups.ReentrantCallbackGroup(),
            'place': rclpy.callback_groups.ReentrantCallbackGroup(),
            'give': rclpy.callback_groups.ReentrantCallbackGroup(),
            'take': rclpy.callback_groups.ReentrantCallbackGroup()
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

        self.give_server = ActionServer(
            self.node, GiveObject, 'give_object',
            execute_callback=self.execute_give, callback_group=self.cb_groups['give'])

        self.take_server = ActionServer(
            self.node, TakeObject, 'take_object',
            execute_callback=self.execute_take, callback_group=self.cb_groups['take'])

        # Publisher: fires when an object has been placed at 'shared' → wakes up the waiting arm
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
        self.logger.info(f"📦 ---> Received Pick Object Action for: '{req_arm}' <---")
        result = PickObject.Result()
        feedback = PickObject.Feedback()
        
        arm_group, _ = get_arm_config(req_arm, self.logger)
        if not arm_group:
            self.safe_abort(goal_handle); result.success = False; return result
        
        req = goal_handle.request
        target_pose = req.target_pose.pose
        fid = req.target_pose.header.frame_id.lower()
        
        clearance = self._get_offset(fid, 'approach_clearance')
        z_offset = self._get_offset(fid, 'pick_z_offset')
        open_w = self.node.get_parameter('gripper_open_width').value
        grasp_w = self.node.get_parameter('gripper_grasp_width').value
        
        if fid in PREDEFINED_TARGETS:
            px, py, pz = PREDEFINED_TARGETS[fid]
            target_pose.position.x, target_pose.position.y, target_pose.position.z = float(px), float(py), float(pz)
            req.target_pose.header.frame_id = WORLD_FRAME
        
        apply_top_down_orientation(target_pose)
        grasp_z = target_pose.position.z + z_offset
        pre_grasp_z = grasp_z + clearance
        
        # 1. Open Gripper
        self.logger.info(f"[{arm_group}] Step 0: Opening Gripper")
        await self.robot_control_api.send_gripper_goal_async(arm_group, width=open_w)

        # --- ROBUSTNESS: Intermediate Waypoint for Shared Zone ---
        if req.target_pose.header.frame_id == "shared":
            self.logger.info(f"[{arm_group}] Collaborative target detected. Moving to MIDWAY joint waypoint first.")
            midway_joints = MIDWAY_POSE_VALUES_RIGHT if "franka1" in arm_group else MIDWAY_POSE_VALUES_LEFT
            await self.robot_control_api.send_moveit_goal_async(arm_group, joint_target=midway_joints, planner="PTP")

        # 2. Approach (PTP)
        self.logger.info(f"[{arm_group}] Step 1: Approach (PTP)")
        pre_grasp = copy.deepcopy(req.target_pose)
        pre_grasp.pose.position.z = pre_grasp_z
        if not await self.robot_control_api.send_moveit_goal_async(arm_group, target_pose=pre_grasp, planner="PTP"):
            self.logger.error(f"[{arm_group}] Step 1 (Approach) FAILED")
            self.safe_abort(goal_handle); result.success = False; return result
            
        # 3. Descent (LIN)
        self.logger.info(f"[{arm_group}] Step 2: Descent (LIN)")
        grasp_pose = copy.deepcopy(req.target_pose)
        grasp_pose.pose.position.z = grasp_z
        if not await self.robot_control_api.send_moveit_goal_async(arm_group, target_pose=grasp_pose, planner="LIN"):
            self.logger.error(f"[{arm_group}] Step 2 (Descent) FAILED")
            self.safe_abort(goal_handle); return result

        # 4. Grasp
        self.logger.info(f"[{arm_group}] Step 3: Grasping")
        await self.robot_control_api.send_gripper_goal_async(arm_group, width=grasp_w)
        
        # 5. Lift (LIN)
        self.logger.info(f"[{arm_group}] Step 4: Lift (LIN)")
        await self.robot_control_api.send_moveit_goal_async(arm_group, target_pose=pre_grasp, planner="LIN")
            
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

        clearance = self._get_offset(fid, 'approach_clearance')
        z_offset = self._get_offset(fid, 'place_z_offset')
        open_w = self.node.get_parameter('gripper_open_width').value
        
        apply_top_down_orientation(place_pose)
        final_place_z = place_pose.position.z + z_offset
        pre_place_z = final_place_z + clearance
        
        pre_place = copy.deepcopy(req.place_pose)
        pre_place.pose.position.z = pre_place_z
        
        # 1. Approach
        self.logger.info(f"[{arm_group}] Step 1: Approach (PTP)")
        if not await self.robot_control_api.send_moveit_goal_async(arm_group, target_pose=pre_place, planner="PTP"):
            self.logger.error(f"[{arm_group}] Step 1 (Approach) FAILED")
            self.safe_abort(goal_handle); return result
            
        # 2. Descent
        self.logger.info(f"[{arm_group}] Step 2: Descent (LIN)")
        place_target = copy.deepcopy(req.place_pose)
        place_target.pose.position.z = final_place_z
        if not await self.robot_control_api.send_moveit_goal_async(arm_group, target_pose=place_target, planner="LIN"):
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
        await self.robot_control_api.send_moveit_goal_async(arm_group, target_pose=pre_place, planner="LIN")
             
        result.success = True
        self.safe_succeed(goal_handle)
        return result

    def execute_give(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"🎁 ---> [GIVE] Action for: '{req_arm}' <---")
        result = GiveObject.Result()
        
        arm_group, tcp_frame = get_arm_config(req_arm, self.logger)
        target_pose = goal_handle.request.handover_pose.pose
        fid = goal_handle.request.handover_pose.header.frame_id.lower()
        
        if fid in PREDEFINED_TARGETS:
            px, py, pz = PREDEFINED_TARGETS[fid]
            target_pose.position.x = float(px)
            target_pose.position.y = float(py)
            target_pose.position.z = float(pz)
            self.logger.info(f"📍 Give Target '{fid}' resolved to {WORLD_FRAME}: ({px}, {py}, {pz})")
        
        apply_donor_handover_orientation(target_pose)
        
        give_pose = copy.deepcopy(goal_handle.request.handover_pose)
        give_pose.header.frame_id = WORLD_FRAME
        give_pose.pose = target_pose

        donor_x_offset = self.node.get_parameter('handover_donor_x_offset').value
        pre_give_pose = copy.deepcopy(give_pose)
        pre_give_pose.pose.position.x += donor_x_offset
        
        if not self.robot_control_api.send_pose_goal(arm_group, pre_give_pose, tcp_frame, is_handover=True):
            self.safe_abort(goal_handle)
            return result

        self._donor_pre_pos_ready.set()
        
        timeout = self.node.get_parameter('handover_timeout_sec').value
        if not self._recipient_pre_pos_ready.wait(timeout=timeout):
            self._donor_pre_pos_ready.clear()
            self.safe_abort(goal_handle)
            return result

        if not self.robot_control_api.send_pose_goal(arm_group, give_pose, tcp_frame, planner="LIN", is_handover=True):
            self._donor_pre_pos_ready.clear()
            self.safe_abort(goal_handle)
            return result

        self._donor_ready.set()
        
        if not self._recipient_ready.wait(timeout=timeout):
            self._donor_ready.clear()
            self._donor_pre_pos_ready.clear()
            self.safe_abort(goal_handle)
            return result

        open_w = self.node.get_parameter('gripper_open_width').value
        self.robot_control_api.send_gripper_goal(arm_group, width=open_w)
        
        if not self._recipient_grasped.wait(timeout=timeout):
            self.logger.warning("[Handshake] recipient never signaled GRASPED.")

        self.robot_control_api.send_pose_goal(arm_group, pre_give_pose, tcp_frame, planner="LIN", is_handover=True)

        self._donor_ready.clear()
        self._donor_pre_pos_ready.clear()
        
        self.safe_succeed(goal_handle)
        result.success = True
        return result

    def execute_take(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"🤝 ---> [TAKE] Action for: '{req_arm}' <---")
        result = TakeObject.Result()
        
        arm_group, tcp_frame = get_arm_config(req_arm, self.logger)
        target_pose = goal_handle.request.handover_pose.pose
        fid = goal_handle.request.handover_pose.header.frame_id.lower()

        if fid in PREDEFINED_TARGETS:
            px, py, pz = PREDEFINED_TARGETS[fid]
            target_pose.position.x = float(px)
            target_pose.position.y = float(py)
            target_pose.position.z = float(pz)
            self.logger.info(f"📍 Take Target '{fid}' resolved to {WORLD_FRAME}: ({px}, {py}, {pz})")
            
        apply_recipient_handover_orientation(target_pose)

        take_pose = copy.deepcopy(goal_handle.request.handover_pose)
        take_pose.header.frame_id = WORLD_FRAME
        take_pose.pose = target_pose
        
        recipient_x_offset = self.node.get_parameter('handover_recipient_x_offset').value
        pre_take_pose = copy.deepcopy(take_pose)
        pre_take_pose.pose.position.x += recipient_x_offset

        if not self.robot_control_api.send_pose_goal(arm_group, pre_take_pose, tcp_frame, is_handover=True):
            self.safe_abort(goal_handle)
            return result
        
        self._recipient_pre_pos_ready.set()
        
        timeout = self.node.get_parameter('handover_timeout_sec').value
        if not self._donor_pre_pos_ready.wait(timeout=timeout):
            self._recipient_pre_pos_ready.clear()
            self.safe_abort(goal_handle)
            return result

        if not self._donor_ready.wait(timeout=timeout):
             self._recipient_pre_pos_ready.clear()
             self.safe_abort(goal_handle)
             return result

        if not self.robot_control_api.send_pose_goal(arm_group, take_pose, tcp_frame, planner="LIN", is_handover=True):
            self._recipient_pre_pos_ready.clear()
            self.safe_abort(goal_handle)
            return result

        self._recipient_ready.set()
        grasp_w = self.node.get_parameter('gripper_grasp_width').value
        self.robot_control_api.send_gripper_goal(arm_group, width=grasp_w)
        
        if not self.robot_control_api.check_grasp(arm_group):
            self._recipient_ready.clear()
            self._recipient_pre_pos_ready.clear()
            self.safe_abort(goal_handle)
            return result

        self._recipient_grasped.set()
        self.robot_control_api.send_pose_goal(arm_group, pre_take_pose, tcp_frame, planner="LIN", is_handover=True)

        self._recipient_ready.clear()
        self._recipient_grasped.clear()
        self._recipient_pre_pos_ready.clear()
        
        self.safe_succeed(goal_handle)
        result.success = True
        return result
