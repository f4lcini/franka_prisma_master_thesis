import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, PlanningScene
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import copy
import asyncio
import time
import traceback

from .config import parse_error_code, WORLD_FRAME


class RobotControlAPI:
    def __init__(self, node, client_cb_group):
        self.node = node
        self.logger = node.get_logger()
        
        # MoveGroup Action (Client side)
        self.move_group_client = ActionClient(
            self.node, MoveGroup, 'move_action', callback_group=client_cb_group)
        
        # Gripper Action Clients — JTC exposes FollowJointTrajectory
        # action at <controller_name>/follow_joint_trajectory
        self.gripper1_client = ActionClient(
            self.node, FollowJointTrajectory, 'franka1_gripper/follow_joint_trajectory',
            callback_group=client_cb_group)
        self.gripper2_client = ActionClient(
            self.node, FollowJointTrajectory, 'franka2_gripper/follow_joint_trajectory',
            callback_group=client_cb_group)

        self.cartesian_client = self.node.create_client(
            GetCartesianPath, 'compute_cartesian_path', callback_group=client_cb_group)
        self.trajectory_executor = ActionClient(
            self.node, ExecuteTrajectory, 'execute_trajectory', callback_group=client_cb_group)

        self.logger.info("⏳ Checking backends (MoveGroup, Grippers)...")
        if not self.move_group_client.wait_for_server(timeout_sec=1.0):
             self.logger.warning("⚠️ MoveGroup (/move_action) not ready yet. Will retry during execution.")
        
        # Gripper Action Servers
        if not self.gripper1_client.wait_for_server(timeout_sec=2.0):
            self.logger.warning("⚠️ franka1_gripper/follow_joint_trajectory not ready yet.")
        if not self.gripper2_client.wait_for_server(timeout_sec=2.0):
            self.logger.warning("⚠️ franka2_gripper/follow_joint_trajectory not ready yet.")

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
            req.max_velocity_scaling_factor = 0.2
        if req.max_acceleration_scaling_factor < 0.01:
            req.max_acceleration_scaling_factor = 0.2

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
        self.logger.info(f"🦾 Sending Gripper Action for {arm_group} (width: {width}m)...")
        
        client = self.gripper1_client if arm_group == "franka1_arm" else self.gripper2_client
        prefix = "franka1" if arm_group == "franka1_arm" else "franka2"
        
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = [f"{prefix}_fr3_finger_joint1", f"{prefix}_fr3_finger_joint2"]
        traj.header.stamp = self.node.get_clock().now().to_msg()
        
        point = JointTrajectoryPoint()
        pos = float(width / 2.0)   # each finger travels half the total width
        point.positions  = [pos, pos]
        point.velocities = [0.0, 0.0]
        # Reduce time to 0.5s for faster response in simulation
        point.time_from_start.sec     = 0
        point.time_from_start.nanosec = 500000000 
        traj.points = [point]
        goal.trajectory = traj

        # Relax tolerances so the action returns SUCCESS even if physics stalls the fingers
        goal.goal_time_tolerance.sec = 0
        goal.goal_time_tolerance.nanosec = 500000000
        
        try:
            goal_handle = await client.send_goal_async(goal)
            if not goal_handle.accepted:
                self.logger.error(f"❌ Gripper goal REJECTED for {arm_group}")
                return False
            
            result = await goal_handle.get_result_async()
            self.logger.info(f"✅ Gripper goal completed (Symmetrical/Fast) for {arm_group}")
            return True
        except Exception as e:
            self.logger.error(f"❌ Gripper action exception: {e}")
            return False

    def publish_planning_scene(self):
        """Pushes latest joint states to the planning scene to ensure inter-arm collision awareness."""
        msg = PlanningScene()
        msg.is_diff = True
        msg.robot_state.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        for name, pos in self.latest_joint_states.items():
            msg.robot_state.joint_state.name.append(name)
            msg.robot_state.joint_state.position.append(pos)
        self.collision_pub.publish(msg)
        # --- HARDENING: Small sleep to ensure MoveIt processes the scene update before the plan request arrives ---
        time.sleep(0.1)

    async def send_pose_goal_custom(self, group_name, pose_stamped, tcp_frame, planner="PTP", is_handover=False):
        self.publish_planning_scene() # Sync state before planning
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
                self.logger.error(f"❌ {planner} Goal REJECTED by MoveGroup for {group_name}.")
                return False
            result = await goal_handle.get_result_async()
            error_val = result.result.error_code.val
            if error_val == 1:
                self.logger.info(f"✅ MoveIt {planner} SUCCESS for {group_name}!")
                return True
            else:
                error_str = parse_error_code(error_val)
                # --- HARDENING: FALLBACK TO OMPL FOR IK FAILURES ---
                if error_val == -1 and planner != "OMPL":
                    self.logger.warning(f"⚠️ {planner} failed IK for {group_name}. Falling back to OMPL...")
                    return await self.send_pose_goal_custom(group_name, pose_stamped, tcp_frame, planner="OMPL", is_handover=is_handover)
                
                self.logger.error(f"❌ MoveIt {planner} FAILED: {error_str} ({error_val}) for {group_name}")
                return False
        except Exception as e:
            self.logger.error(f"❌ Pilz Exception: {e}")
            return False

    async def send_pose_goal(self, group_name, pose_stamped, tcp_frame, is_handover=False):
        self.publish_planning_scene() # Sync state before planning
        self.logger.info(f"📍 Requesting Cartesian Move for {group_name} to frame {pose_stamped.header.frame_id}")
        
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = group_name
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 1
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
                self.logger.error("❌ Goal REJECTED by MoveGroup server.")
                return False
            self.logger.info("⏳ Goal accepted! Planning/Moving in progress...")
            result = await goal_handle.get_result_async()
            error_val = result.result.error_code.val
            error_str = parse_error_code(error_val)
            if error_val == 1:
                self.logger.info(f"✅ MoveIt Cartesian execution SUCCESS for {group_name}!")
                return True
            else:
                self.logger.error(f"❌ MoveIt Execution FAILED: {error_str} ({error_val})")
                return False
        except Exception as e:
            self.logger.error(f"❌ Critical Exception during send_pose_goal: {e}")
            self.logger.error(traceback.format_exc())
            return False

    async def send_cartesian_move(self, group_name, waypoints, tcp_frame):
        self.logger.info(f"📏 Requesting Linear (Cartesian) Move for {group_name} with {len(waypoints)} points...")
        
        req = GetCartesianPath.Request()
        req.header.frame_id = WORLD_FRAME
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.group_name = group_name
        req.link_name = tcp_frame
        req.waypoints = waypoints
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        
        if not self.cartesian_client.wait_for_service(timeout_sec=2.0):
             self.logger.error("❌ Cartesian service not available!")
             return False
             
        res = await self.cartesian_client.call_async(req)
        if res.fraction < 0.5:
            self.logger.error(f"❌ Cartesian path planning failed (only {res.fraction*100:.1f}% planned)")
            return False
            
        self.logger.info(f"✅ Cartesian path computed ({res.fraction*100:.1f}%). Executing...")
        
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        
        try:
            handle = await self.trajectory_executor.send_goal_async(goal)
            if not handle.accepted:
                self.logger.error("❌ Trajectory execution goal REJECTED")
                return False
            
            exec_res = await handle.get_result_async()
            if exec_res.result.error_code.val == 1:
                self.logger.info("✅ Cartesian execution SUCCESS!")
                return True
            else:
                self.logger.error(f"❌ Cartesian execution FAILED: {exec_res.result.error_code.val}")
                return False
        except Exception as e:
            self.logger.error(f"❌ Exception in cartesian execution: {e}")
            return False

    def check_grasp(self, arm_group, threshold=0.04, stall_threshold=0.001):
        """
        Calculates the combined effort on the gripper fingers to check for a grasp.
        --- HARDENING: Added temporal verification and POSITION STALL check for Sim/No-Perception ---
        """
        prefix = "franka1" if "franka1" in arm_group else "franka2"
        f1 = f"{prefix}_fr3_finger_joint1"
        f2 = f"{prefix}_fr3_finger_joint2"
        
        # We need to know what width we were AIMING for to check for stall
        # Default grasp width is 0.048 (4.8cm). Cube is 5.0cm.
        target_width = 0.048 
        
        verification_count = 0
        stall_count = 0
        
        for _ in range(5):
            eff1 = self.latest_efforts.get(f1, 0.0)
            eff2 = self.latest_efforts.get(f2, 0.0)
            total_effort = abs(eff1) + abs(eff2)
            
            p1 = self.latest_joint_states.get(f1, None)
            p2 = self.latest_joint_states.get(f2, None)
            
            if p1 is None or p2 is None:
                self.logger.warning(f"⚠️ Gripper joints NOT FOUND. f1={f1} in keys: {f1 in self.latest_joint_states}, f2={f2} in keys: {f2 in self.latest_joint_states}")
                if not self.latest_joint_states:
                    self.logger.warning("Empty joint states!")
                actual_width = 0.0
            else:
                actual_width = p1 + p2
            
            # 1. Effort check (Good for hardware)
            if total_effort > threshold:
                verification_count += 1
            
            # 2. Position Stall check (Good for simulation)
            # If fingers are significantly wider than the target width, they probably hit something.
            if actual_width > (target_width + stall_threshold):
                stall_count += 1
                
            time.sleep(0.1)
            
        is_grasped_effort = verification_count >= 3
        is_grasped_stall = stall_count >= 3
        
        self.logger.info(f"🔍 Grasp Check [{prefix}]: Effort Hit {verification_count}/5 | Position Stall {stall_count}/5 (Width: {actual_width:.3f}m)")
        
        if is_grasped_effort:
            self.logger.info(f"✅ Grasp CONFIRMED via effort threshold ({threshold})")
            return True
        if is_grasped_stall:
            self.logger.info(f"✅ Grasp CONFIRMED via position stall (Obstruction at {actual_width:.3f}m vs Target {target_width}m)")
            return True
            
        return False
    def calculate_midway_pose(self, arm_group, target_pose_stamped):
        """
        Dynamically calculates a 'Midway' pose between current position and target.
        Ensures a high Z-offset for safety.
        """
        prefix = "franka1" if "franka1" in arm_group else "franka2"
        tcp_frame = f"{prefix}_fr3_hand_tcp"
        
        # Get current pose (Mocking if missing joint states, but usually available)
        # For simplicity in this env, we'll interpolate based on a 'Rough Home' center
        # or just use a higher Z-buffer.
        
        midway = copy.deepcopy(target_pose_stamped)
        midway.header.frame_id = WORLD_FRAME
        
        # Logic: Move 50% towards target in XY, but keep Z high (e.g. 0.5m)
        # This keeps the arm 'poised' for the next action.
        
        current_x = 1.0 if prefix == "franka1" else 0.0
        current_y = 0.5
        
        target_x = midway.pose.position.x
        target_y = midway.pose.position.y
        
        midway.pose.position.x = (current_x + target_x) / 2.0
        midway.pose.position.y = (current_y + target_y) / 2.0
        midway.pose.position.z = 0.50 # High safe Z
        
        apply_top_down_orientation(midway.pose)
        return midway

def apply_top_down_orientation(pose):
    # Standard top-down grasp (pointing down)
    pose.orientation.x = 1.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0
