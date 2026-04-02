import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import MtcPickObject
import copy

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 - registers the transform for PoseStamped
from geometry_msgs.msg import PoseStamped

# -----------------------------------------------------------------
# TEST_MODE: bypass YOLO/blackboard and send a hardcoded world-frame
# pose directly to the MTC server.
# Controlled at runtime via ROS parameter 'test_mode' (default: False).
# -----------------------------------------------------------------
TEST_POSE_WORLD = (0.50, 0.00, 0.026)   # red_cube position (updated to match test_skills.py)

class MtcPickActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Execute MTC Pick", action_name="/mtc_pick_object"):
        super().__init__(name=name)
        self.action_name = action_name
        self.node = None
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None
        self.tf_buffer = None
        self.tf_listener = None
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="target_pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="active_arm", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError:
            self.logger.error(f"[{self.name}] ROS 2 node not found in setup kwargs. Cannot create Action Client!")
            return False

        # Declare test_mode parameter (can be set via launch file)
        if not self.node.has_parameter('test_mode'):
            self.node.declare_parameter('test_mode', False)

        self.action_client = ActionClient(self.node, MtcPickObject, self.action_name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.logger.info(f"[{self.name}] Waiting for MTC Pick server at '{self.action_name}'...")
        self.action_client.wait_for_server(timeout_sec=1.0)
        return True

    def initialise(self):
        self.logger.info(f"[{self.name}] Initializing MTC Pick...")
        self.send_goal_future = None
        self.get_result_future = None

        target_arm = "left_arm"  # default

        test_mode = self.node.get_parameter('test_mode').get_parameter_value().bool_value

        if test_mode:
            # ── TEST MODE ───────────────────────────────────────────────
            # Bypass YOLO + TF pipeline. Send the red_cube world-frame pose
            # directly so we can validate MTC/IK independently.
            self.logger.warning(f"[{self.name}] *** TEST_MODE ACTIVE — using hardcoded world pose {TEST_POSE_WORLD} ***")
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'world'
            target_pose.header.stamp = self.node.get_clock().now().to_msg()
            target_pose.pose.position.x = 1.10
            target_pose.pose.position.y = 0.20
            target_pose.pose.position.z = 0.225
            # Top-down orientation: gripper pointing down (180° around X)
            target_pose.pose.orientation.w = 0.0
            target_pose.pose.orientation.x = 1.0
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = 0.0
            # ─────────────────────────────────────────────────────────────
        else:
            try:
                target_pose = self.blackboard.target_pose
                if target_pose is None:
                    raise ValueError("Target pose is None")
            except AttributeError:
                self.logger.error(f"[{self.name}] No 'target_pose' found on the blackboard!")
                self.status = py_trees.common.Status.FAILURE
                return

            if hasattr(self.blackboard, "active_arm"):
                target_arm = self.blackboard.active_arm

            # --- Transform pose to world frame (safety fallback) ---
            # NOTE: object_localization_node outputs frame_id='world' directly via
            # TF-free extrinsic math. This block should be a no-op in normal operation
            # (source_frame == 'world'). It is kept as a fallback for legacy nodes.
            source_frame = target_pose.header.frame_id
            if source_frame and source_frame != 'world':
                try:
                    self.logger.warning(
                        f"[{self.name}] Pose frame='{source_frame}' != 'world'. "
                        "TF legacy fallback triggered."
                    )
                    target_pose = self.tf_buffer.transform(
                        target_pose, 'world',
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    self.logger.info(
                        f"[{self.name}] Transformed to world: "
                        f"({target_pose.pose.position.x:.3f}, "
                        f"{target_pose.pose.position.y:.3f}, "
                        f"{target_pose.pose.position.z:.3f})"
                    )
                except Exception as e:
                    self.logger.error(
                        f"[{self.name}] TF transform failed: {e}. Sending in original frame."
                    )


        goal_msg = MtcPickObject.Goal()
        goal_msg.arm = target_arm
        goal_msg.target_pose = copy.deepcopy(target_pose)
        goal_msg.approach_distance = 0.15

        if not self.action_client.server_is_ready():
            self.logger.error(f"[{self.name}] Action server {self.action_name} is OFF!")
            self.status = py_trees.common.Status.FAILURE
            return

        self.logger.info(f"[{self.name}] Sending action goal asynchronously...")
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.status = py_trees.common.Status.RUNNING


    def update(self):
        if hasattr(self, 'status') and self.status == py_trees.common.Status.FAILURE:
            return py_trees.common.Status.FAILURE

        if self.get_result_future is None:
            if self.send_goal_future and self.send_goal_future.done():
                goal_handle = self.send_goal_future.result()
                if not goal_handle.accepted:
                    self.logger.error(f"[{self.name}] MTC Pick goal REJECTED by server!")
                    return py_trees.common.Status.FAILURE
                self.logger.info(f"[{self.name}] MTC Pick goal ACCEPTED. Waiting for execution...")
                self.get_result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING
        
        if self.get_result_future.done():
            result = self.get_result_future.result().result
            if result.success:
                self.logger.info(f"[{self.name}] MTC Pick SUCCEEDED: {result.message}")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"[{self.name}] MTC Pick FAILED: {result.message}")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.send_goal_future = None
        self.get_result_future = None