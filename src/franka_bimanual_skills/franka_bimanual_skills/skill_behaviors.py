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

from .config import PREDEFINED_TARGETS, WORLD_FRAME, READY_POSE_VALUES_LEFT, READY_POSE_VALUES_RIGHT, MIDWAY_POSE_VALUES_LEFT, MIDWAY_POSE_VALUES_RIGHT, get_arm_config, apply_top_down_orientation, apply_donor_handover_orientation, apply_recipient_handover_orientation, parse_error_code

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
            'home': rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
            'pick': rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
            'place': rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
            'give': rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
            'take': rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
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

    def execute_home(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"🔄 [Paradigm 1] Received Unified MoveHome for: '{req_arm}'")
        result = MoveHome.Result()
        feedback = MoveHome.Feedback()
        
        arm_group, tcp_frame = get_arm_config(req_arm, self.logger)
        pose_target = goal_handle.request.pose_name.lower() or "ready"
        
        if arm_group == "franka1_arm":
            ready_values = MIDWAY_POSE_VALUES_RIGHT if pose_target == "midway" else READY_POSE_VALUES_RIGHT
        else:
            ready_values = MIDWAY_POSE_VALUES_LEFT if pose_target == "midway" else READY_POSE_VALUES_LEFT
            
        feedback.status = f"Planning ParallelMove (Joints) for {req_arm}"
        self.safe_publish_feedback(goal_handle, feedback)

        # Call the unified ParallelMove action (Synchronous API)
        success = self.robot_control_api.send_joint_goal(arm_group, ready_values, planner="ompl")

        result.success = success
        if success:
            open_w = self.node.get_parameter('gripper_open_width').value if self.node.has_parameter('gripper_open_width') else 0.08
            self.logger.info(f"👐 Opening gripper for {arm_group} to {open_w}m...")
            self.robot_control_api.send_gripper_goal(arm_group, width=open_w, max_effort=10.0)
            self.safe_succeed(goal_handle)
        else:
            self.safe_abort(goal_handle)
        return result

    def execute_pick(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"📦 ---> Received Pick Object Action for: '{req_arm}' <---")
        result = PickObject.Result()
        feedback = PickObject.Feedback()
        
        clearance = self.node.get_parameter('approach_clearance').value
        z_offset = self.node.get_parameter('pick_z_offset').value
        open_w = self.node.get_parameter('gripper_open_width').value
        grasp_w = self.node.get_parameter('gripper_grasp_width').value
        pause_s = self.node.get_parameter('safety_pause_short').value
        pause_l = self.node.get_parameter('safety_pause_long').value

        arm_group, tcp_frame = get_arm_config(req_arm, self.logger)
        if not arm_group:
            self.safe_abort(goal_handle)
            result.success = False
            return result
        
        req = goal_handle.request
        target_pose = req.target_pose.pose
        fid = req.target_pose.header.frame_id.lower()
        
        if fid in PREDEFINED_TARGETS:
            px, py, pz = PREDEFINED_TARGETS[fid]
            target_pose.position.x, target_pose.position.y, target_pose.position.z = px, py, pz
            req.target_pose.header.frame_id = WORLD_FRAME # Normalize to world
        
        apply_top_down_orientation(target_pose)
        grasp_z = target_pose.position.z + z_offset
        pre_grasp_z = grasp_z + clearance
        
        self.logger.info(f"🎯 Pick Target (Final Z={grasp_z:.3f})")

        # 1. Open Gripper
        feedback.status = f"Pre-Step: Opening Gripper fully ({open_w}m)"
        self.safe_publish_feedback(goal_handle, feedback)
        
        for attempt in range(3):
            if self.robot_control_api.send_gripper_goal(arm_group, width=open_w):
                break
            time.sleep(0.5)
        time.sleep(1.0) # Wait for simulation physics

        # 2. Approach
        pre_grasp = copy.deepcopy(req.target_pose)
        pre_grasp.pose.position.z = pre_grasp_z
        apply_top_down_orientation(pre_grasp.pose)
        
        feedback.status = "Step 1/4: Moving to Pre-Grasp"
        self.safe_publish_feedback(goal_handle, feedback)
        if not self.robot_control_api.send_pose_goal(arm_group, pre_grasp, tcp_frame, planner="PTP"):
            self.safe_abort(goal_handle)
            result.success = False
            return result
        time.sleep(0.5) # Settling time
            
        # 3. Descent (LIN)
        feedback.status = f"Step 2/4: Vertical Descent to Z={grasp_z}"
        self.safe_publish_feedback(goal_handle, feedback)
        grasp_pose = copy.deepcopy(req.target_pose)
        grasp_pose.pose.position.z = grasp_z
        if not self.robot_control_api.send_pose_goal(arm_group, grasp_pose, tcp_frame, planner="LIN"):
            self.safe_abort(goal_handle)
            return result
        time.sleep(0.5) # Final settle before grasp

        # 4. Grasp
        feedback.status = f"Step 3/4: Closing Gripper to {grasp_w}m"
        self.safe_publish_feedback(goal_handle, feedback)
        self.robot_control_api.send_gripper_goal(arm_group, width=grasp_w)
        time.sleep(pause_s)
        
        if not self.robot_control_api.check_grasp(arm_group):
            self.logger.error(f"❌ Grasp FAILED for {arm_group}.")
            self.safe_abort(goal_handle)
            result.success = False
            return result
        
        # 5. Lift (LIN)
        feedback.status = "Step 4/4: Vertical Lift (Pilz LIN)"
        self.safe_publish_feedback(goal_handle, feedback)
        if not self.robot_control_api.send_pose_goal(arm_group, pre_grasp, tcp_frame, planner="LIN"):
            self.safe_abort(goal_handle)
            return result
            
        result.success = True
        self.safe_succeed(goal_handle)
        return result

    def execute_place(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"📥 ---> Received Place Object Action for: '{req_arm}' <---")
        result = PlaceObject.Result()
        
        clearance = self.node.get_parameter('approach_clearance').value
        z_offset = self.node.get_parameter('place_z_offset').value
        open_w = self.node.get_parameter('gripper_open_width').value
        pause_s = self.node.get_parameter('safety_pause_short').value

        arm_group, tcp_frame = get_arm_config(req_arm, self.logger)
        req = goal_handle.request
        place_pose = req.place_pose.pose
        apply_top_down_orientation(place_pose)
        
        final_place_z = place_pose.position.z + z_offset
        pre_place_z = final_place_z + clearance
        
        pre_place = copy.deepcopy(req.place_pose)
        pre_place.pose.position.z = pre_place_z
        apply_top_down_orientation(pre_place.pose)
        
        # 1. Approach
        if not self.robot_control_api.send_pose_goal(arm_group, pre_place, tcp_frame):
            self.safe_abort(goal_handle)
            return result
            
        # 2. Descent
        place_target = copy.deepcopy(req.place_pose)
        place_target.pose.position.z = final_place_z
        if not self.robot_control_api.send_pose_goal(arm_group, place_target, tcp_frame, planner="LIN"):
            self.safe_abort(goal_handle)
            return result
        time.sleep(0.5) # Settle before release
            
        # 3. Release
        self.robot_control_api.send_gripper_goal(arm_group, width=open_w)
        
        # Handover signal
        fid = req.place_pose.header.frame_id.lower()
        if fid in PREDEFINED_TARGETS:
            px, py, pz = PREDEFINED_TARGETS[fid]
            place_pose.position.x, place_pose.position.y, place_pose.position.z = px, py, pz
            req.place_pose.header.frame_id = WORLD_FRAME # Normalize to world

        if fid == 'shared':
            msg = Bool()
            msg.data = True
            self._handover_ready_pub.publish(msg)
        
        # 4. Retreat
        self.robot_control_api.send_pose_goal(arm_group, pre_place, tcp_frame, planner="LIN")
             
        result.success = True
        self.safe_succeed(goal_handle)
        return result

    def execute_give(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.logger.info(f"🎁 ---> [GIVE] Action for: '{req_arm}' <---")
        result = GiveObject.Result()
        
        arm_group, tcp_frame = get_arm_config(req_arm, self.logger)
        target_pose = goal_handle.request.handover_pose.pose
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
