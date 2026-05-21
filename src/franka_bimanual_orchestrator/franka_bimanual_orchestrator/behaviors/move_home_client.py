import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import MoveHome
import time

class MoveHomeClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Execute Move Home", action_name="/move_home", prefix="left_", target_pose=None):
        super().__init__(name=name)
        self.action_name = action_name
        self.prefix = prefix
        self.target_pose = target_pose
        self.node = None
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=f"{prefix}active_arm", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="handover_ready", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="metrics_logger", access=py_trees.common.Access.READ)

    def _log_metrics(self, success):
        try:
            if hasattr(self.blackboard, 'metrics_logger') and self.blackboard.metrics_logger:
                end_t = time.time()
                target_arm = getattr(self.blackboard, f"{self.prefix}active_arm", self.prefix.replace("_", ""))
                self.blackboard.metrics_logger.log_action(target_arm, "MOVE_HOME", self.start_t, end_t, success)
                
                if success:
                    self.blackboard.metrics_logger.log_execution(True)
                else:
                    self.blackboard.metrics_logger.log_execution(False)
        except Exception:
            pass

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(self.node, MoveHome, self.action_name)
        return True

    def initialise(self):
        self.logged = False
        self.start_t = time.time()
        self.send_goal_future = None
        self.get_result_future = None
        
        target_arm = getattr(self.blackboard, f"{self.prefix}active_arm", "left_arm")
        # Priority: 1. constructor argument, 2. blackboard key, 3. default 'ready'
        target_pose = self.target_pose or getattr(self.blackboard, f"{self.prefix}target_pose_name", "ready")
        
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
                    if not getattr(self, 'logged', False):
                        self._log_metrics(False)
                        self.logged = True
                    return py_trees.common.Status.FAILURE
                self.get_result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING
        
        if self.get_result_future.done():
            result = self.get_result_future.result().result
            if not getattr(self, 'logged', False):
                self._log_metrics(result.success)
                self.logged = True
            return py_trees.common.Status.SUCCESS if result.success else py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.send_goal_future = None
        self.get_result_future = None
