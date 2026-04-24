import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import MoveHome

class MoveHomeClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Execute Move Home", action_name="/move_home", prefix="left_"):
        super().__init__(name=name)
        self.action_name = action_name
        self.prefix = prefix
        self.node = None
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=f"{prefix}active_arm", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"{prefix}target_pose_name", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(self.node, MoveHome, self.action_name)
        return True

    def initialise(self):
        self.send_goal_future = None
        self.get_result_future = None
        
        target_arm = getattr(self.blackboard, f"{self.prefix}active_arm", "left_arm")
        target_pose = getattr(self.blackboard, f"{self.prefix}target_pose_name", "ready")
        
        goal_msg = MoveHome.Goal()
        goal_msg.arm = target_arm
        goal_msg.pose_name = target_pose

        self.node.get_logger().info(f"[{self.name}] MoveHome: {target_arm} to {target_pose}")
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)

    def update(self):
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
