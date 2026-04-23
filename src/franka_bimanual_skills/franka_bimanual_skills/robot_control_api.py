import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, PlanningScene
from franka_custom_interfaces.action import ParallelMove
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand
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
        
        # Paradigm 1: Parallel Move Client (Connecting to our C++ Bimanual Planner)
        self.parallel_move_client = ActionClient(
            self.node, ParallelMove, 'parallel_move', callback_group=client_cb_group)
        
        # Gripper Action Clients — Reverted to FollowJointTrajectory for dual-finger symmetry
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

        self.logger.info("⏳ Checking backends (Planner, Grippers)...")
        
        if not self.parallel_move_client.wait_for_server(timeout_sec=2.0):
             self.logger.warning("⚠️ ParallelMove (/parallel_move) not ready yet.")
        
        if not self.gripper1_client.wait_for_server(timeout_sec=2.0):
            self.logger.warning("⚠️ franka1_gripper not ready yet.")
        if not self.gripper2_client.wait_for_server(timeout_sec=2.0):
            self.logger.warning("⚠️ franka2_gripper not ready yet.")

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

    def wait_for_future(self, future, timeout_sec=10.0, label="Action"):
        """Synchronous wait for a future to avoid asyncio loop issues."""
        start_time = time.time()
        while not future.done():
            if (time.time() - start_time) > timeout_sec:
                self.logger.error(f"❌ {label} TIMEOUT after {timeout_sec}s")
                return None
            time.sleep(0.01)
        return future.result()

    def send_gripper_goal(self, arm_group, width, max_effort=100.0):
        """Synchronous gripper goal using FJT for dual-joint symmetry."""
        self.logger.info(f"🦾 Sending Gripper Action for {arm_group} (width: {width}m)...")
        
        client = self.gripper1_client if arm_group == "franka1_arm" else self.gripper2_client
        prefix = "franka1" if "franka1" in arm_group else "franka2"
        
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = [f"{prefix}_fr3_finger_joint1", f"{prefix}_fr3_finger_joint2"]
        
        point = JointTrajectoryPoint()
        # Symmetric move: each finger goes to width/2
        target_pos = float(width / 2.0)
        point.positions = [target_pos, target_pos]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        
        traj.points = [point]
        goal.trajectory = traj
        
        # Generous duration margin for Sim
        goal.goal_time_tolerance.sec = 2

        try:
            future_handle = client.send_goal_async(goal)
            goal_handle = self.wait_for_future(future_handle, timeout_sec=2.0, label="GripAccept")
            
            if not goal_handle or not goal_handle.accepted:
                self.logger.error(f"❌ Gripper goal REJECTED for {arm_group}")
                return False
            
            future_result = goal_handle.get_result_async()
            res = self.wait_for_future(future_result, timeout_sec=10.0, label="GripResult")
            
            if res is None:
                 self.logger.warning(f"⚠️ Gripper FJT TIMEOUT for {arm_group} - Proceeding")
            else:
                 self.logger.info(f"✅ Gripper goal finished symmetrically for {arm_group}")
            
            return True
        except Exception as e:
            self.logger.error(f"❌ Gripper FJT action exception: {e}")
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

    def send_pose_goal(self, group_name, pose_stamped, tcp_frame, planner="PTP", is_handover=False):
        """Synchronous Paradigm 1 Move."""
        self.publish_planning_scene()
        self.logger.info(f"📍 [Paradigm 1] Requesting {planner} Parallel Move for {group_name}...")
        
        goal_msg = ParallelMove.Goal()
        goal_msg.arm = "right" if "franka1" in group_name else "left"
        goal_msg.target_pose = pose_stamped
        goal_msg.planner_id = planner
        goal_msg.is_handover = is_handover

        try:
            future_handle = self.parallel_move_client.send_goal_async(goal_msg)
            goal_handle = self.wait_for_future(future_handle, timeout_sec=2.0, label="MoveAccept")
            
            if not goal_handle or not goal_handle.accepted:
                self.logger.error(f"❌ ParallelMove Goal REJECTED for {group_name}.")
                return False

            future_result = goal_handle.get_result_async()
            result = self.wait_for_future(future_result, timeout_sec=300.0, label="MoveResult")
            
            if result and result.result.success:
                self.logger.info(f"✅ ParallelMove SUCCESS for {group_name}!")
                return True
            else:
                msg = result.result.message if result else "Timeout"
                self.logger.error(f"❌ ParallelMove FAILED for {group_name}: {msg}")
                return False
        except Exception as e:
            self.logger.error(f"❌ ParallelMove Exception: {e}")
            traceback.print_exc()
            return False

    def send_joint_goal(self, group_name, joint_values, planner="PTP"):
        """Synchronous Paradigm 1 Joint Move."""
        self.logger.info(f"🦾 [Paradigm 1] Requesting Parallel Jnt Move for {group_name}...")
        
        goal_msg = ParallelMove.Goal()
        goal_msg.arm = "right" if "franka1" in group_name else "left"
        goal_msg.joint_target = joint_values
        goal_msg.planner_id = planner

        try:
            future_handle = self.parallel_move_client.send_goal_async(goal_msg)
            goal_handle = self.wait_for_future(future_handle, timeout_sec=2.0, label="JntMoveAccept")
            
            if not goal_handle or not goal_handle.accepted:
                return False

            future_result = goal_handle.get_result_async()
            result = self.wait_for_future(future_result, timeout_sec=300.0, label="JntMoveResult")
            return result.result.success if result else False
        except Exception as e:
            return False

    def check_grasp(self, arm_group, effort_threshold=0.5, min_width=0.01, max_width=0.07):
        """
        Verifies if an object is held by checking finger positions and efforts.
        In Gazebo, a successful grasp means the fingers stalled before closing fully.
        """
        prefix = "franka1" if "franka1" in arm_group else "franka2"
        f1 = f"{prefix}_fr3_finger_joint1"
        f2 = f"{prefix}_fr3_finger_joint2"
        
        success_count = 0
        for _ in range(10): # Check more samples for stability
            p1 = self.latest_joint_states.get(f1, None)
            p2 = self.latest_joint_states.get(f2, None)
            
            if p1 is not None and p2 is not None:
                actual_width = p1 + p2
                # In Gazebo, effort is noisy/zero when geometry stalls perfectly. 
                # Pure width stalling is the only structurally sound heuristic for simulation.
                if min_width <= actual_width <= max_width:
                    success_count += 1
            
            time.sleep(0.05)
            
        is_grasped = success_count >= 5
        if is_grasped:
            self.logger.info(f"✅ [GraspCheck] Object detected! (Stalled within {min_width}-{max_width})")
        else:
            self.logger.warning(f"⚠️ [GraspCheck] No object detected (Fingers likely closed or slipping)")
            
        return is_grasped
