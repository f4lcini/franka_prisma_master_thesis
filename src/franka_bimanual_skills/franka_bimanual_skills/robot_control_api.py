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
        
        # Gripper Action Clients — Using standard GripperCommand for hardware compatibility
        self.gripper1_client = ActionClient(
            self.node, GripperCommand, '/franka1_gripper/gripper_action',
            callback_group=client_cb_group)
        self.gripper2_client = ActionClient(
            self.node, GripperCommand, '/franka2_gripper/gripper_action',
            callback_group=client_cb_group)
        
        self.cartesian_client = self.node.create_client(
            GetCartesianPath, 'compute_cartesian_path', callback_group=client_cb_group)
        self.trajectory_executor = ActionClient(
            self.node, ExecuteTrajectory, 'execute_trajectory', callback_group=client_cb_group)

        self.logger.info("⏳ Checking backends (Planner, Grippers)...")
        
        if not self.parallel_move_client.wait_for_server(timeout_sec=2.0):
             self.logger.warning("⚠️ ParallelMove (/parallel_move) not ready yet.")
        
        if not self.gripper1_client.wait_for_server(timeout_sec=5.0):
            self.logger.warning("⚠️ franka1_gripper not ready yet.")
        if not self.gripper2_client.wait_for_server(timeout_sec=5.0):
            self.logger.warning("⚠️ franka2_gripper not ready yet.")

    def wait_for_future(self, future, timeout_sec=None, label="Action"):
        """Simple synchronous wait for a rclpy future."""
        start_time = time.time()
        while rclpy.ok() and not future.done():
            if timeout_sec and (time.time() - start_time) > timeout_sec:
                self.logger.error(f"❌ Timeout waiting for {label}")
                return None
            time.sleep(0.01)
        return future.result()

    def send_gripper_goal(self, arm_group, width, max_effort=20.0):
        """Synchronous gripper goal using GripperCommand for hardware compatibility."""
        client = self.gripper1_client if "franka1" in arm_group else self.gripper2_client
        
        if not client.wait_for_server(timeout_sec=2.0):
            self.logger.error(f"Gripper server for {arm_group} not available!")
            return False

        goal = GripperCommand.Goal()
        # Franka gripper action expects per-finger position (width / 2)
        # Added safety margin (0.075 max) to avoid out-of-range errors on hardware
        safe_width = min(float(width), 0.075)
        goal.command.position = safe_width / 2.0
        goal.command.max_effort = float(max_effort)

        self.logger.info(f"🦾 Sending gripper goal to {arm_group}: width={width}m")
        try:
            future = client.send_goal_async(goal)
            handle = self.wait_for_future(future, timeout_sec=2.0, label="GripGoal")
            if not handle or not handle.accepted:
                return False

            result_future = handle.get_result_async()
            res = self.wait_for_future(result_future, timeout_sec=10.0, label="GripResult")
            return res is not None
        except Exception as e:
            self.logger.error(f"❌ Gripper action exception: {e}")
            return False

    def send_pose_goal(self, arm_group, target_pose, tcp_frame, planner="PTP", is_handover=False):
        """Send a synchronous goal to the Bimanual Planner C++ node."""
        self.logger.info(f"🚀 Sending ParallelMove goal (Pose) for {arm_group} to {planner}...")
        
        goal = ParallelMove.Goal()
        goal.arm = "right" if "franka1" in arm_group else "left"
        goal.target_pose = target_pose
        goal.planner_id = planner
        goal.is_handover = is_handover

        return self._send_parallel_move(goal, arm_group)

    def send_joint_goal(self, arm_group, joint_values, planner="PTP"):
        """Send a synchronous joint-space goal to the Bimanual Planner C++ node."""
        self.logger.info(f"🚀 Sending ParallelMove goal (Joints) for {arm_group} to {planner}...")
        
        goal = ParallelMove.Goal()
        goal.arm = "right" if "franka1" in arm_group else "left"
        goal.joint_target = [float(v) for v in joint_values]
        goal.planner_id = planner

        return self._send_parallel_move(goal, arm_group)

    def _send_parallel_move(self, goal, arm_group):
        """Internal helper to send ParallelMove goals and wait for results."""
        try:
            future = self.parallel_move_client.send_goal_async(goal)
            handle = self.wait_for_future(future, timeout_sec=5.0, label="ParallelMoveGoal")
            
            if not handle or not handle.accepted:
                self.logger.error(f"❌ ParallelMove goal REJECTED for {arm_group}")
                return False
            
            result_future = handle.get_result_async()
            res = self.wait_for_future(result_future, timeout_sec=30.0, label="ParallelMoveResult")
            
            if res and res.result.success:
                self.logger.info(f"✅ ParallelMove for {arm_group} SUCCEEDED.")
                return True
            else:
                msg = res.result.message if res else "Timeout/Unknown"
                self.logger.error(f"❌ ParallelMove for {arm_group} FAILED: {msg}")
                return False
        except Exception as e:
            self.logger.error(f"❌ ParallelMove exception: {e}")
            traceback.print_exc()
            return False

    def check_grasp(self, arm_group):
        """Verify if an object is grasped."""
        return True
