#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time

from franka_custom_interfaces.action import MtcMoveHome, MtcPickObject, MtcPlaceObject
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint, RobotTrajectory
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from datetime import datetime

import copy
import time
import traceback
import asyncio

MOVEIT_ERROR_CODES = {
    1: "SUCCESS",
    99999: "FAILURE",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "START_STATE_VIOLATES_VELOCITY_LIMITS",
    -13: "GOAL_IN_COLLISION",
    -14: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -15: "GOAL_VIOLATES_VELOCITY_LIMITS",
    -16: "GOAL_CONSTRAINTS_VIOLATED",
    -17: "INVALID_GROUP_NAME",
    -18: "INVALID_GOAL_CONSTRAINTS",
    -19: "INVALID_ROBOT_STATE",
    -20: "INVALID_LINK_NAME",
    -21: "INVALID_OBJECT_NAME",
    -31: "NO_IK_SOLUTION"
}

class SimpleMoveItServer(Node):
    def __init__(self):
        super().__init__('simple_moveit_server')
        # CRITICAL FIX: To avoid deadlock in ROS 2 Humble when an action calls another action,
        # we separate the Server and Client into different Callback Groups.
        self._server_cb_group = ReentrantCallbackGroup()
        self._client_cb_group = ReentrantCallbackGroup()
        
        # MoveGroup Action (Client side)
        self.move_group_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=self._client_cb_group)
        
        # Gripper Actions (Client side)
        self.gripper1_client = ActionClient(self, FollowJointTrajectory, 'franka1_gripper/follow_joint_trajectory', callback_group=self._client_cb_group)
        self.gripper2_client = ActionClient(self, FollowJointTrajectory, 'franka2_gripper/follow_joint_trajectory', callback_group=self._client_cb_group)
            
        # Cartesian Path Service & Execution
        self.cartesian_client = self.create_client(
            GetCartesianPath, 'compute_cartesian_path', callback_group=self._client_cb_group)
        self.trajectory_executor = ActionClient(
            self, ExecuteTrajectory, 'execute_trajectory', callback_group=self._client_cb_group)

        self.get_logger().info("==============================================")
        self.get_logger().info(" Python Action Servers Starting (Strategic CBG)... ")
        self.get_logger().info("==============================================")

        # Wait for clients at startup (non-blocking enough to allow executor to start)
        self.get_logger().info("⏳ Checking backends (MoveGroup, Grippers)...")
        if not self.move_group_client.wait_for_server(timeout_sec=1.0):
             self.get_logger().warning("⚠️ MoveGroup (/move_action) not ready yet. Will retry during execution.")
        
        if not self.gripper1_client.wait_for_server(timeout_sec=0.5):
             self.get_logger().warning("⚠️ Gripper 1 not ready yet.")
        if not self.gripper2_client.wait_for_server(timeout_sec=0.5):
             self.get_logger().warning("⚠️ Gripper 2 not ready yet.")

        # Actions (Server side) - ADVERTISE IMMEDIATELY
        self.home_server = ActionServer(
            self, MtcMoveHome, 'mtc_move_home',
            execute_callback=self.execute_home, callback_group=self._server_cb_group)
        
        self.pick_server = ActionServer(
            self, MtcPickObject, 'mtc_pick_object',
            execute_callback=self.execute_pick, callback_group=self._server_cb_group)
            
        self.place_server = ActionServer(
            self, MtcPlaceObject, 'mtc_place_object',
            execute_callback=self.execute_place, callback_group=self._server_cb_group)
            
        self.get_logger().info("✅ Python Servers (MoveHome, Pick, Place) Advertised!")

        # --- PICK & PLACE PARAMETERS ---
        self.declare_parameter('approach_clearance', 0.1)
        self.declare_parameter('pick_z_offset', 0.105) # Centroid(0.225) + 10.5cm = 0.33m
        self.declare_parameter('place_z_offset', 0.055)   # Direct target at 0.3m (as requested)
        self.declare_parameter('gripper_open_width', 0.08)
        self.declare_parameter('gripper_grasp_width', 0.048)
        self.declare_parameter('safety_pause_short', 0.5)
        self.declare_parameter('safety_pause_long', 1.0)



    def parse_error_code(self, code_val):
        return MOVEIT_ERROR_CODES.get(code_val, f"UNKNOWN_ERROR_CODE_{code_val}")

    def get_arm_config(self, request_arm):
        if request_arm == "left_arm":
            return "franka2_arm", "franka2_fr3_hand_tcp"
        elif request_arm == "right_arm":
            return "franka1_arm", "franka1_fr3_hand_tcp"
        else:
            self.get_logger().error(f"❌ Unrecognized arm request: '{request_arm}'")
            return None, None


    def _apply_safety_limits(self, req):
        if req.max_velocity_scaling_factor < 0.01:
            req.max_velocity_scaling_factor = 0.2
        if req.max_acceleration_scaling_factor < 0.01:
            req.max_acceleration_scaling_factor = 0.2


    def _apply_top_down_orientation(self, pose):
        """Forces the TCP to point straight down (180deg rotation around X)."""
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

    async def send_gripper_goal(self, arm_group, width, max_effort=20.0):

        self.get_logger().info(f"🦾 Requesting Symmetrical Gripper Action for {arm_group} (width: {width})...")
        client = self.gripper1_client if arm_group == "franka1_arm" else self.gripper2_client
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            f"{'franka1' if arm_group == 'franka1_arm' else 'franka2'}_fr3_finger_joint1",
            f"{'franka1' if arm_group == 'franka1_arm' else 'franka2'}_fr3_finger_joint2"
        ]
        
        point = JointTrajectoryPoint()
        # Each finger moves half the width
        pos = width / 2.0
        point.positions = [pos, pos]
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        
        goal_msg.trajectory.points = [point]
        
        try:
            # We must use wait_for_server to be safe
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Gripper Action Server not available")
                return False

            goal_handle_future = client.send_goal_async(goal_msg)
            
            # Use rclpy wait to avoid asyncio loop issues
            import time
            start_t = time.time()
            while not goal_handle_future.done() and (time.time() - start_t < 2.0):
                time.sleep(0.01)
                
            goal_handle = goal_handle_future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Gripper Goal Rejected")
                return False
            
            self.get_logger().info("Gripper Goal Accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            
            
            start_t = time.time()
            # Wait for execution to finish
            while not result_future.done() and (time.time() - start_t < 10.0):
                time.sleep(0.1)
                
            if result_future.done():
                self.get_logger().info("Gripper Goal Completed SUCCESSFULLY")
            else:
                self.get_logger().warn("Gripper Goal Timeout (Continuing anyway)")
                
            return True
        except Exception as e:
            self.get_logger().error(f"Error in send_gripper_goal: {e}")
            return False

    async def send_pose_goal_custom(self, group_name, pose_stamped, tcp_frame, planner="PTP"):
        self.get_logger().info(f"📍 Requesting {planner} Move for {group_name} using Pilz...")
        
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = group_name
        req.allowed_planning_time = 5.0
        self._apply_safety_limits(req)
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = planner

        
        c = Constraints()
        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = tcp_frame
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        sp.dimensions = [0.001, 0.001, 0.001] # Pilz prefers tiny boxes or pure Pose goals
        bv.primitives.append(sp)
        bv.primitive_poses.append(pose_stamped.pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        
        oc = OrientationConstraint()
        oc.header = pose_stamped.header
        oc.link_name = tcp_frame
        oc.orientation = pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1 # Loosened slightly for IK success
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
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
            self.get_logger().error(f"❌ Pilz Exception: {e}")
            return False

    async def send_pose_goal(self, group_name, pose_stamped, tcp_frame):
        self.get_logger().info(f"📍 Requesting Cartesian Move for {group_name} to frame {pose_stamped.header.frame_id}")
        
        # Server availability guaranteed at startup — no blocking call here

        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = group_name
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 1
        self._apply_safety_limits(req)
        
        # PILZ INTEGRATION: Using PTP by default for pose goals
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
        oc.absolute_x_axis_tolerance = 0.05 # Tightened to force top-down
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
                self.get_logger().error("❌ Goal REJECTED by MoveGroup server.")
                return False
            self.get_logger().info("⏳ Goal accepted! Planning/Moving in progress...")
            result = await goal_handle.get_result_async()
            error_val = result.result.error_code.val
            error_str = self.parse_error_code(error_val)
            if error_val == 1:
                self.get_logger().info(f"✅ MoveIt Cartesian execution SUCCESS for {group_name}!")
                return True
            else:
                self.get_logger().error(f"❌ MoveIt Execution FAILED: {error_str} ({error_val})")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Critical Exception during send_pose_goal: {e}")
            self.get_logger().error(traceback.format_exc())
            return False

    async def send_cartesian_move(self, group_name, waypoints, tcp_frame):
        self.get_logger().info(f"📏 Requesting Linear (Cartesian) Move for {group_name} with {len(waypoints)} points...")
        
        # 1. Compute Path
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = group_name
        req.link_name = tcp_frame
        req.waypoints = waypoints
        req.max_step = 0.01
        req.jump_threshold = 0.0 # Disabled
        req.avoid_collisions = True
        
        if not self.cartesian_client.wait_for_service(timeout_sec=2.0):
             self.get_logger().error("❌ Cartesian service not available!")
             return False
             
        res = await self.cartesian_client.call_async(req)
        if res.fraction < 0.5:
            self.get_logger().error(f"❌ Cartesian path planning failed (only {res.fraction*100:.1f}% planned)")
            return False
            
        self.get_logger().info(f"✅ Cartesian path computed ({res.fraction*100:.1f}%). Executing...")
        
        # 2. Execute Path
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        
        try:
            handle = await self.trajectory_executor.send_goal_async(goal)
            if not handle.accepted:
                self.get_logger().error("❌ Trajectory execution goal REJECTED")
                return False
            
            exec_res = await handle.get_result_async()
            if exec_res.result.error_code.val == 1:
                self.get_logger().info("✅ Cartesian execution SUCCESS!")
                return True
            else:
                self.get_logger().error(f"❌ Cartesian execution FAILED: {exec_res.result.error_code.val}")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Exception in cartesian execution: {e}")
            return False

    async def execute_home(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.get_logger().info(f"🔄 ---> Received MoveHome Action for: '{req_arm}' <---")
        result = MtcMoveHome.Result()
        feedback = MtcMoveHome.Feedback()
        
        arm_group, tcp_frame = self.get_arm_config(req_arm)
        if not arm_group:
            self.get_logger().error("Aborting MoveHome: Invalid Arm Group.")
            goal_handle.abort()
            result.success = False
            result.message = f"Invalid arm group: '{req_arm}'"
            return result
        
        # Feedback: starting
        feedback.status = f"Planning MoveHome for {arm_group}"
        feedback.completion_percentage = 10.0
        goal_handle.publish_feedback(feedback)
            
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = arm_group
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 10
        
        ready_values = [0.0, -0.785398, 0.0, -2.35619, 0.0, 1.570796, 0.785398]
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
        
        # GLOBAL SAFETY GUARDS for Pilz
        self._apply_safety_limits(req)
        
        # PILZ PTP for Home
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"
        
        goal_msg.request = req
        
        self.get_logger().info(f"📍 Requesting Joint Move (READY Pose) for {arm_group}...")
        feedback.status = f"Executing MoveHome for {arm_group}"
        feedback.completion_percentage = 50.0
        goal_handle.publish_feedback(feedback)
        
        try:
            handle = await self.move_group_client.send_goal_async(goal_msg)
            if handle.accepted:
                self.get_logger().info("⏳ Goal accepted! Planning and moving...")
                res = await handle.get_result_async()
                error_val = res.result.error_code.val
                error_str = self.parse_error_code(error_val)
                success = (error_val == 1)
                if success:
                    self.get_logger().info(f"✅ MoveHome execution SUCCESS for {arm_group}!")
                    result.message = f"MoveHome completed for {arm_group}"
                else:
                    self.get_logger().error(f"❌ MoveHome FAILED: {error_str} ({error_val})")
                    result.message = f"MoveHome failed: {error_str} ({error_val})"
            else:
                self.get_logger().error("❌ MoveHome Goal REJECTED by MoveGroup server.")
                success = False
                result.message = "MoveHome goal rejected by MoveGroup"
        except Exception as e:
            self.get_logger().error(f"❌ Critical Exception during MoveHome: {e}")
            self.get_logger().error(traceback.format_exc())
            success = False
            result.message = f"Exception during MoveHome: {e}"

        result.success = success
        feedback.status = "Completed" if success else "Failed"
        feedback.completion_percentage = 100.0
        goal_handle.publish_feedback(feedback)
        
        if success:
            self.get_logger().info(f"👐 Opening gripper for {arm_group}...")
            await self.send_gripper_goal(arm_group, width=0.08, max_effort=10.0)
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def execute_pick(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.get_logger().info(f"📦 ---> Received Pick Object Action for: '{req_arm}' <---")
        result = MtcPickObject.Result()
        feedback = MtcPickObject.Feedback()
        
        # Load Parameters
        clearance = self.get_parameter('approach_clearance').value
        z_offset = self.get_parameter('pick_z_offset').value
        open_w = self.get_parameter('gripper_open_width').value
        grasp_w = self.get_parameter('gripper_grasp_width').value
        pause_s = self.get_parameter('safety_pause_short').value
        pause_l = self.get_parameter('safety_pause_long').value

        arm_group, tcp_frame = self.get_arm_config(req_arm)
        if not arm_group:
            goal_handle.abort()
            result.success = False
            result.message = f"Invalid arm group: '{req_arm}'"
            return result
        
        req = goal_handle.request
        target_pose = req.target_pose.pose
        self._apply_top_down_orientation(target_pose)
        
        # Final Z based on Request + Offset
        grasp_z = target_pose.position.z + z_offset
        pre_grasp_z = grasp_z + clearance
        
        self.get_logger().info(f"🎯 Pick Target (Base Z={target_pose.position.z:.3f}, Final Z={grasp_z:.3f})")

        # PRE-STEP: Ensure gripper is wide open
        feedback.status = f"Pre-Step: Opening Gripper fully ({open_w}m)"
        goal_handle.publish_feedback(feedback)
        await self.send_gripper_goal(arm_group, width=open_w, max_effort=10.0)

        # STEP 1: Pre-Grasp
        pre_grasp = copy.deepcopy(req.target_pose)
        pre_grasp.pose.position.z = pre_grasp_z
        self._apply_top_down_orientation(pre_grasp.pose)
        
        self.get_logger().info(f"🚀 Approach to Z={pre_grasp_z:.3f}...")
        
        feedback.status = "Step 1/4: Moving to Pre-Grasp"
        feedback.completion_percentage = 25.0
        goal_handle.publish_feedback(feedback)
        if not await self.send_pose_goal(arm_group, pre_grasp, tcp_frame):
            goal_handle.abort()
            result.success = False
            result.message = "Pre-Grasp move failed"
            return result
            
        # STEP 2: Vertical Descent (LIN)
        feedback.status = f"Step 2/4: Vertical Descent to Z={grasp_z}"
        feedback.completion_percentage = 50.0
        goal_handle.publish_feedback(feedback)
        
        grasp_pose = copy.deepcopy(req.target_pose)
        grasp_pose.pose.position.z = grasp_z
        
        self.get_logger().info(f"📍 Executing Linear Descent (LIN) to Z={grasp_z}...")
        if not await self.send_pose_goal_custom(arm_group, grasp_pose, tcp_frame, planner="LIN"):
            goal_handle.abort()
            result.success = False
            result.message = "Vertical descent (LIN) failed"
            return result

        time.sleep(pause_s)
            
        # STEP 3: Grasp
        feedback.status = f"Step 3/4: Closing Gripper to {grasp_w}m"
        feedback.completion_percentage = 75.0
        goal_handle.publish_feedback(feedback)
        await self.send_gripper_goal(arm_group, width=grasp_w, max_effort=30.0)

        time.sleep(pause_l) 
            
        # STEP 4: Vertical Lift (LIN)
        feedback.status = "Step 4/4: Vertical Lift (Pilz LIN)"
        feedback.completion_percentage = 90.0
        goal_handle.publish_feedback(feedback)
        if not await self.send_pose_goal_custom(arm_group, pre_grasp, tcp_frame, planner="LIN"):
            goal_handle.abort()
            result.success = False
            result.message = "Vertical lift (LIN) failed"
            return result
            
        time.sleep(pause_s)

        feedback.status = "Pick sequence completed"
        feedback.completion_percentage = 100.0
        goal_handle.publish_feedback(feedback)
        self.get_logger().info("🎉 Strategic Pick Sequence SUCCESS!")
        result.success = True
        result.message = f"Pick successful with {arm_group}"
        goal_handle.succeed()
        return result

    async def execute_place(self, goal_handle):
        req_arm = goal_handle.request.arm
        self.get_logger().info(f"📥 ---> Received Place Object Action for: '{req_arm}' <---")
        result = MtcPlaceObject.Result()
        feedback = MtcPlaceObject.Feedback()
        
        # Load Parameters
        clearance = self.get_parameter('approach_clearance').value
        z_offset = self.get_parameter('place_z_offset').value
        open_w = self.get_parameter('gripper_open_width').value
        pause_s = self.get_parameter('safety_pause_short').value
        pause_l = self.get_parameter('safety_pause_long').value

        arm_group, tcp_frame = self.get_arm_config(req_arm)
        if not arm_group:
            goal_handle.abort()
            result.success = False
            result.message = f"Invalid arm group: '{req_arm}'"
            return result
            
        req = goal_handle.request
        place_pose = req.place_pose.pose
        self._apply_top_down_orientation(place_pose)
        
        # Final Z based on Request + Offset
        final_place_z = place_pose.position.z + z_offset
        pre_place_z = final_place_z + clearance
        
        self.get_logger().info(f"🎯 Place Target (Base Z={place_pose.position.z:.3f}, Final Z={final_place_z:.3f})")

        # Step 1: Pre-Place
        pre_place = copy.deepcopy(req.place_pose)
        pre_place.pose.position.z = pre_place_z
        self._apply_top_down_orientation(pre_place.pose)
        
        feedback.status = f"Step 1/4: Moving to Pre-Place (Z={pre_place_z:.3f})"
        goal_handle.publish_feedback(feedback)
        if not await self.send_pose_goal(arm_group, pre_place, tcp_frame):
            goal_handle.abort()
            result.success = False
            result.message = "Pre-Place move failed"
            return result
            
        time.sleep(pause_s)
            
        # Step 2: Linear Descent (LIN)
        feedback.status = f"Step 2/4: Vertical Descent to Z={final_place_z}"
        goal_handle.publish_feedback(feedback)
        
        place_target = copy.deepcopy(req.place_pose)
        place_target.pose.position.z = final_place_z
        
        self.get_logger().info(f"📍 Executing Linear Descent (LIN) to Z={final_place_z} for Place...")
        if not await self.send_pose_goal_custom(arm_group, place_target, tcp_frame, planner="LIN"):
            goal_handle.abort()
            result.success = False
            result.message = "Vertical descent (LIN) failed for Place"
            return result
            
        time.sleep(pause_s)

        # Step 3: Open Gripper
        feedback.status = f"Step 3/4: Opening Gripper ({open_w}m)"
        goal_handle.publish_feedback(feedback)
        await self.send_gripper_goal(arm_group, width=open_w, max_effort=10.0)
        
        time.sleep(pause_l) # Wait for physical release
        
        # Step 4: Vertical Retreat (LIN)
        feedback.status = "Step 4/4: Vertical Retreat"
        goal_handle.publish_feedback(feedback)
        if not await self.send_pose_goal_custom(arm_group, pre_place, tcp_frame, planner="LIN"):
             self.get_logger().warn("Retreat failed, but object placed.")
             
        time.sleep(pause_s)
             
        result.success = True
        result.message = f"Place successful with {arm_group}"
        self.get_logger().info("🎉 Strategic Place Sequence SUCCESS!")
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMoveItServer()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=12)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down gracefully...")
    except Exception as e:
        node.get_logger().fatal(f"Unhandled Execution Exception: {e}")
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
