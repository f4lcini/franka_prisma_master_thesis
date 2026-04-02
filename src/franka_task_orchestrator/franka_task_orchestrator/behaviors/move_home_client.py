import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import MtcMoveHome

class MoveHomeClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Execute MTC Move Home", action_name="/mtc_move_home"):
        super().__init__(name=name)
        self.action_name = action_name
        self.node = None
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="active_action", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="active_arm", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError:
            self.logger.error(f"[{self.name}] ROS 2 node not found in setup kwargs. Cannot create Action Client!")
            return False

        self.action_client = ActionClient(self.node, MtcMoveHome, self.action_name)
        self.logger.info(f"[{self.name}] Waiting for MTC MoveHome server at '{self.action_name}'...")
        self.action_client.wait_for_server(timeout_sec=1.0)
        return True

    def initialise(self):
        self.logger.info(f"[{self.name}] Initializing MTC MoveHome...")
        self.send_goal_future = None
        self.get_result_future = None
        
        target_arm = "left_arm" # default fallback
        if hasattr(self.blackboard, "active_arm"):
            target_arm = self.blackboard.active_arm
        
        # Server only accepts 'left_arm' or 'right_arm'
        if target_arm not in ("left_arm", "right_arm"):
            self.logger.warning(f"[{self.name}] active_arm='{target_arm}' is not valid. Falling back to 'left_arm'.")
            target_arm = "left_arm"

        goal_msg = MtcMoveHome.Goal()
        goal_msg.arm = target_arm

        if not self.action_client.server_is_ready():
            self.logger.error(f"[{self.name}] Action server {self.action_name} is OFF!")
            self.status = py_trees.common.Status.FAILURE
            return

        self.logger.info(f"[{self.name}] Sending action goal asynchronously for arm: {target_arm}...")
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.status = py_trees.common.Status.RUNNING

    def update(self):
        if hasattr(self, 'status') and self.status == py_trees.common.Status.FAILURE:
            return py_trees.common.Status.FAILURE

        if self.get_result_future is None:
            if self.send_goal_future and self.send_goal_future.done():
                goal_handle = self.send_goal_future.result()
                if not goal_handle.accepted:
                    self.logger.error(f"[{self.name}] MTC MoveHome goal REJECTED by server!")
                    return py_trees.common.Status.FAILURE
                self.logger.info(f"[{self.name}] MTC MoveHome goal ACCEPTED. Waiting for execution...")
                self.get_result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING
        
        if self.get_result_future.done():
            result = self.get_result_future.result().result
            if result.success:
                self.logger.info(f"[{self.name}] MTC MoveHome SUCCEEDED: {result.message}")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"[{self.name}] MTC MoveHome FAILED: {result.message}")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.send_goal_future = None
        self.get_result_future = None
