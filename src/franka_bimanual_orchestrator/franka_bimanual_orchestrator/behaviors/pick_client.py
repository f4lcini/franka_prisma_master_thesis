import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import PickObject
import copy

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 - registers the transform for PoseStamped
from geometry_msgs.msg import PoseStamped

# pose directly to the skill server.
# Controlled at runtime via ROS parameter 'test_mode' (default: False).
# -----------------------------------------------------------------
TEST_POSE_TABLE = (0.4, -0.25, 0.0)   # Default target_object position on table

class PickActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Execute Pick", action_name="/pick_object", prefix="left_", target_name=None):
        super().__init__(name=name)
        self.action_name = action_name
        self.prefix = prefix
        self.target_name_override = target_name
        self.node = None
        # ... rest of init ...
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None
        self.tf_buffer = None
        self.tf_listener = None
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=f"{prefix}target_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"{prefix}target_pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"{prefix}active_arm", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="handover_ready", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        # ... setup remains same ...
        try:
            self.node = kwargs['node']
        except KeyError:
            self.logger.error(f"[{self.name}] ROS 2 node not found in setup kwargs!")
            return False

        if not self.node.has_parameter('test_mode'):
            self.node.declare_parameter('test_mode', False)

        self.action_client = ActionClient(self.node, PickObject, self.action_name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.logger.info(f"[{self.name}] Waiting for Pick server at '{self.action_name}'...")
        self.action_client.wait_for_server(timeout_sec=1.0)
        return True

    def initialise(self):
        self.logger.info(f"[{self.name}] Initializing Pick...")
        target_arm = "any"
        target_label = "none"
        
        # Priority: Constructor Override -> Blackboard (Safe Access)
        if self.target_name_override:
            target_label = self.target_name_override
        else:
            try:
                target_label = getattr(self.blackboard, f"{self.prefix}target_name")
            except (AttributeError, KeyError):
                target_label = "none"
        
        try:
            # Safe read from blackboard
            target_arm = getattr(self.blackboard, f"{self.prefix}active_arm")
        except (AttributeError, KeyError):
            # Default fallback if not on blackboard
            target_arm = "right_arm" if "right" in self.prefix else "left_arm"
            self.logger.info(f"[{self.name}] Blackboard arm missing, using default: {target_arm}")

        test_mode = self.node.get_parameter('test_mode').get_parameter_value().bool_value

        # PREDEFINED TARGET SUPPORT
        if target_label in ["base_pose", "shared", "box", "target_object", "box_ws_sx", "box_ws_dx"]:
            self.logger.info(f"[{self.name}] Using predefined target: '{target_label}'")
            target_pose = PoseStamped()
            target_pose.header.frame_id = target_label
            target_pose.header.stamp = self.node.get_clock().now().to_msg()
            # Server handles the rest
        elif test_mode:
            self.logger.warning(f"[{self.name}] *** TEST_MODE ACTIVE ***")
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'table'
            target_pose.header.stamp = self.node.get_clock().now().to_msg()
            target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z = TEST_POSE_TABLE
            target_pose.pose.orientation.w = 0.0
            target_pose.pose.orientation.x = 1.0
        else:
            try:
                target_pose = getattr(self.blackboard, f"{self.prefix}target_pose")
            except AttributeError:
                self.logger.error(f"[{self.name}] No '{self.prefix}target_pose' found!")
                self.status = py_trees.common.Status.FAILURE
                return

        goal_msg = PickObject.Goal()
        goal_msg.arm = target_arm
        goal_msg.target_pose = copy.deepcopy(target_pose)
        goal_msg.approach_distance = 0.15

        self.logger.info(f"[{self.name}] Sending Pick Goal for {target_arm} to {goal_msg.target_pose.header.frame_id}")
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.status = py_trees.common.Status.RUNNING

    def update(self):
        if hasattr(self, 'status') and self.status == py_trees.common.Status.FAILURE:
            return py_trees.common.Status.FAILURE

        if self.get_result_future is None:
            if self.send_goal_future and self.send_goal_future.done():
                goal_handle = self.send_goal_future.result()
                if not goal_handle.accepted:
                    return py_trees.common.Status.FAILURE
                self.get_result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING
        
        if self.get_result_future.done():
            result = self.get_result_future.result().result
            if result.success:
                target_label = getattr(self.blackboard, f"{self.prefix}target_name", "none")
                if target_label == "shared":
                    self.blackboard.handover_ready = False
                    self.logger.info(f"[{self.name}] Shared Pick complete. Handover flag reset.")
                return py_trees.common.Status.SUCCESS
            
            # --- Robustness: Return FAILURE instead of infinite retry ---
            error_msg = result.message.lower() if hasattr(result, 'message') else ""
            self.logger.error(f"[{self.name}] Pick FAILED: {error_msg}")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.send_goal_future = None
        self.get_result_future = None