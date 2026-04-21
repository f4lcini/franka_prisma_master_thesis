import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, PlanningScene
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import copy
import time
import traceback

from .config import parse_error_code, WORLD_FRAME


class RobotControlAPI:
    def __init__(self, node, client_cb_group):
        self.node = node
        self.logger = node.get_logger()
        
        # MoveGroup Action (Client side)
        self.move_group_client = ActionClient(
            self.node, MoveGroup, '/move_action', callback_group=client_cb_group)
        
        # Gripper Actions (Client side - Hardware Namespaces)
        self.gripper1_client = ActionClient(self.node, GripperCommand, '/franka1/franka_gripper/gripper_action', callback_group=client_cb_group)
        self.gripper2_client = ActionClient(self.node, GripperCommand, '/franka2/franka_gripper/gripper_action', callback_group=client_cb_group)
            
        # Cartesian Path Service & Execution
        self.cartesian_client = self.node.create_client(
            GetCartesianPath, 'compute_cartesian_path', callback_group=client_cb_group)
        self.trajectory_executor = ActionClient(
            self.node, ExecuteTrajectory, 'execute_trajectory', callback_group=client_cb_group)

        self.logger.info("⏳ Checking backends (MoveGroup, Grippers)...")
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
             self.logger.warning("⚠️ MoveGroup (/move_action) not ready yet. Will retry during execution.")
        
        if not self.gripper1_client.wait_for_server(timeout_sec=2.0):
             self.logger.warning("⚠️ Gripper 1 not ready yet.")
        if not self.gripper2_client.wait_for_server(timeout_sec=2.0):
             self.logger.warning("⚠️ Gripper 2 not ready yet.")

        # --- DYNAMIC COLLISION & FORCE TRACKING ---
        self.latest_joint_states = {}
        self.latest_efforts = {}
        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10, callback_group=client_cb_group)
        self.collision_pub = self.node.create_publisher(PlanningScene, '/planning_scene', 10)

    def joint_state_cb(self, msg):
        for name, pos, eff in zip(msg.name, msg.position, msg.effort):
            self.latest_joint_states[name] = pos
            self.latest_efforts[name] = eff

    def apply_safety_limits(self, req):
        if req.max_velocity_scaling_factor < 0.01:
            req.max_velocity_scaling_factor = 0.5
        if req.max_acceleration_scaling_factor < 0.01:
            req.max_acceleration_scaling_factor = 0.5

    def apply_workspace_constraints(self, req, arm_group, tcp_frame, is_handover=False):
        """Adds path constraints to keep arms in their respective zones."""
        if is_handover:
            return # No constraints during handover dance
            
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = WORLD_FRAME
        pc.link_name = tcp_frame
        
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        
        # Define Safe Volumes (Loosened split at 0.6)
        if arm_group == "franka1_arm":
            # Right Arm: Stay in X > 0.50
            sp.dimensions = [1.5, 2.0, 2.2]
            center_x = 1.25
        else:
            # Left Arm: Stay in X < 0.70
            sp.dimensions = [1.5, 2.0, 2.2]
            center_x = -0.05
            
        pose = Pose()
        pose.position.x = center_x
        pose.position.y = 0.5
        pose.position.z = 0.5
        pose.orientation.w = 1.0
            
        bv.primitives.append(sp)
        bv.primitive_poses.append(pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints.append(pc)
        req.path_constraints = c

    async def send_gripper_goal(self, arm_group, width, max_effort=20.0):
        self.logger.info(f"🦾 Requesting Gripper Action for {arm_group} (width: {width})...")
        client = self.gripper1_client if arm_group == "franka1_arm" else self.gripper2_client
            
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = width
        goal_msg.command.max_effort = max_effort
        
        try:
            if not client.wait_for_server(timeout_sec=5.0):
                self.logger.error("Gripper Action Server not available")
                return False

            goal_handle = await client.send_goal_async(goal_msg)
            if not goal_handle.accepted:
                self.logger.error("Gripper Goal Rejected")
                return False
            
            self.logger.info("Gripper Goal Accepted, waiting for result...")
            await goal_handle.get_result_async()
            return True
        except Exception as e:
            self.logger.error(f"Error in send_gripper_goal: {e}")
            return False

    def publish_planning_scene(self):
        msg = PlanningScene()
        msg.is_diff = True
        msg.robot_state.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        for name, pos in self.latest_joint_states.items():
            msg.robot_state.joint_state.name.append(name)
            msg.robot_state.joint_state.position.append(pos)
        self.collision_pub.publish(msg)
        time.sleep(0.1)

    async def send_pose_goal_custom(self, group_name, pose_stamped, tcp_frame, planner="PTP", is_handover=False):
        self.publish_planning_scene()
        self.logger.info(f"📍 Requesting {planner} Move for {group_name} using Pilz...")
        
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = group_name
        req.allowed_planning_time = 5.0
        self.apply_safety_limits(req)
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = planner

        c = Constraints()
        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = tcp_frame
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        sp.dimensions = [0.001, 0.001, 0.001]
        bv.primitives.append(sp)
        bv.primitive_poses.append(pose_stamped.pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        
        oc = OrientationConstraint()
        oc.header = pose_stamped.header
        oc.link_name = tcp_frame
        oc.orientation = pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)
        
        self.apply_workspace_constraints(req, group_name, tcp_frame, is_handover=is_handover)
        goal_msg.request = req

        try:
            goal_handle = await self.move_group_client.send_goal_async(goal_msg)
            if not goal_handle.accepted:
                self.logger.error(f"❌ {planner} Goal REJECTED.")
                return False
            result = await goal_handle.get_result_async()
            if result.result.error_code.val == 1:
                return True
            else:
                self.logger.error(f"❌ MoveIt {planner} FAILED: {result.result.error_code.val}")
                return False
        except Exception as e:
            self.logger.error(f"❌ Pilz Exception: {e}")
            return False

    async def send_pose_goal(self, group_name, pose_stamped, tcp_frame, is_handover=False):
        self.publish_planning_scene()
        self.logger.info(f"📍 Requesting Move for {group_name} to frame {pose_stamped.header.frame_id}")
        
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = group_name
        req.allowed_planning_time = 5.0
        self.apply_safety_limits(req)
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP" 
        
        c = Constraints()
        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = tcp_frame
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.02]
        bv.primitives.append(sp)
        bv.primitive_poses.append(pose_stamped.pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        
        oc = OrientationConstraint()
        oc.header = pose_stamped.header
        oc.link_name = tcp_frame
        oc.orientation = pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)
        goal_msg.request = req

        try:
            goal_handle = await self.move_group_client.send_goal_async(goal_msg)
            if not goal_handle.accepted:
                return False
            result = await goal_handle.get_result_async()
            return result.result.error_code.val == 1
        except Exception as e:
            self.logger.error(f"❌ Exception in send_pose_goal: {e}")
            return False

    async def send_cartesian_move(self, group_name, waypoints, tcp_frame):
        req = GetCartesianPath.Request()
        req.header.frame_id = WORLD_FRAME
        req.group_name = group_name
        req.link_name = tcp_frame
        req.waypoints = waypoints
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        
        res = await self.cartesian_client.call_async(req)
        if res.fraction < 0.5:
            return False
            
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        handle = await self.trajectory_executor.send_goal_async(goal)
        if not handle.accepted:
            return False
        exec_res = await handle.get_result_async()
        return exec_res.result.error_code.val == 1

    def check_grasp(self, arm_group, threshold=0.1, stall_threshold=0.001):
        """Force-based grasp validation."""
        prefix = "franka1" if "franka1" in arm_group else "franka2"
        f1 = f"{prefix}_fr3_finger_joint1"
        f2 = f"{prefix}_fr3_finger_joint2"
        
        verification_count = 0
        for _ in range(5):
            eff1 = self.latest_efforts.get(f1, 0.0)
            eff2 = self.latest_efforts.get(f2, 0.0)
            if (abs(eff1) + abs(eff2)) > threshold:
                verification_count += 1
            time.sleep(0.1)
        
        is_grasped = verification_count >= 3
        self.logger.info(f"🔍 Grasp Check [{prefix}]: {is_grasped} (effort count {verification_count}/5)")
        return is_grasped
