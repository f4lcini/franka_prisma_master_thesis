import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import PickObject
import copy

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 - registers the transform for PoseStamped
from geometry_msgs.msg import PoseStamped

# -----------------------------------------------------------------
# TEST_MODE: bypass YOLO/blackboard and send a hardcoded world-frame
# pose directly to the MTC server.
# Controlled at runtime via ROS parameter 'test_mode' (default: False).
# -----------------------------------------------------------------
TEST_POSE_WORLD = (1.10, 0.20, 0.225)   # red_cube position

class PickActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Execute Pick", action_name="/pick_object", prefix="left_"):
        super().__init__(name=name)
        self.action_name = action_name
        self.prefix = prefix
        self.node = None
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None
        self.tf_buffer = None
        self.tf_listener = None
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=f"{prefix}target_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"{prefix}active_arm", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="handover_ready", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
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
        try:
            target_arm = getattr(self.blackboard, f"{self.prefix}active_arm")
        except (AttributeError, KeyError):
            pass
        try:
            target_label = getattr(self.blackboard, f"{self.prefix}target_name")
        except (AttributeError, KeyError):
            pass

        test_mode = self.node.get_parameter('test_mode').get_parameter_value().bool_value

        # PREDEFINED TARGET SUPPORT
        if target_label in ["base_pose", "shared", "box"]:
            self.logger.info(f"[{self.name}] Using predefined target: '{target_label}'")
            target_pose = PoseStamped()
            target_pose.header.frame_id = target_label
            target_pose.header.stamp = self.node.get_clock().now().to_msg()
            # Server handles the rest
        elif test_mode:
            self.logger.warning(f"[{self.name}] *** TEST_MODE ACTIVE ***")
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'world'
            target_pose.header.stamp = self.node.get_clock().now().to_msg()
            target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z = TEST_POSE_WORLD
            target_pose.pose.orientation.w = 0.0
            target_pose.pose.orientation.x = 1.0
        else:
            try:
                target_pose = getattr(self.blackboard, f"{self.prefix}target_pose")
            except AttributeError:
                self.logger.error(f"[{self.name}] No '{self.prefix}target_pose' found!")
                self.status = py_trees.common.Status.FAILURE
                return

        if hasattr(self.blackboard, f"{self.prefix}active_arm"):
            target_arm = getattr(self.blackboard, f"{self.prefix}active_arm")

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
                target_label = getattr(self.blackboard, f"{self.prefix}target_label", "none")
                if target_label == "shared":
                    self.blackboard.handover_ready = False
                    self.logger.info(f"[{self.name}] Shared Pick complete. Handover flag reset.")
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.send_goal_future = None
        self.get_result_future = None