import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import GiveObject
import copy
from geometry_msgs.msg import PoseStamped

class GiveActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Execute Give", action_name="/give_object", prefix="left_"):
        super().__init__(name=name)
        self.action_name = action_name
        self.prefix = prefix
        self.node = None
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=f"{prefix}target_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"{prefix}active_arm", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError:
            self.logger.error(f"[{self.name}] ROS 2 node not found in setup kwargs!")
            return False

        self.action_client = ActionClient(self.node, GiveObject, self.action_name)
        self.logger.info(f"[{self.name}] Waiting for Give server...")
        self.action_client.wait_for_server(timeout_sec=1.0)
        return True

    def initialise(self):
        self.logger.info(f"[{self.name}] Initializing Mid-Air GIVE...")
        self.send_goal_future = None
        self.get_result_future = None
        
        target_arm = "any"
        target_loc = "mid_air"
        try:
            target_arm = getattr(self.blackboard, f"{self.prefix}active_arm")
        except (AttributeError, KeyError):
            pass
        try:
            target_loc = getattr(self.blackboard, f"{self.prefix}target_name")
        except (AttributeError, KeyError):
            pass

        goal_msg = GiveObject.Goal()
        goal_msg.arm = target_arm
        goal_msg.handover_pose = PoseStamped()
        goal_msg.handover_pose.header.frame_id = target_loc
        goal_msg.handover_pose.header.stamp = self.node.get_clock().now().to_msg()

        self.logger.info(f"[{self.name}] Sending GIVE Goal for {target_arm} to {target_loc}")
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
            return py_trees.common.Status.SUCCESS if result.success else py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.send_goal_future = None
        self.get_result_future = None
