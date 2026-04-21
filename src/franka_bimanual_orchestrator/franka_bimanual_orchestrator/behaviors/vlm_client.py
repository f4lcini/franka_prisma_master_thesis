import py_trees
import rclpy
import json
from rclpy.action import ActionClient
from franka_custom_interfaces.action import VlmQuery

class VlmActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="VLM Query", action_name="/vlm_query", task_description="Default task"):
        super().__init__(name=name)
        self.action_name = action_name
        self.task_description = task_description
        self.node = None
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="vlm_plan", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = ActionClient(self.node, VlmQuery, self.action_name)
        self.action_client.wait_for_server(timeout_sec=10.0)

    def initialise(self):
        goal_msg = VlmQuery.Goal()
        goal_msg.task_description = self.task_description
        self.blackboard.vlm_plan = None
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)

    def update(self):
        if self.send_goal_future and not self.send_goal_future.done():
            return py_trees.common.Status.RUNNING

        if self.send_goal_future and self.send_goal_future.done() and not self.get_result_future:
            goal_handle = self.send_goal_future.result()
            if not goal_handle.accepted:
                return py_trees.common.Status.FAILURE
            self.get_result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if self.get_result_future and self.get_result_future.done():
            result = self.get_result_future.result().result
            if result.success:
                try:
                    plan_data = json.loads(result.vlm_plan_json)
                    # New structure: vlm_plan is the whole TaskPlan dict
                    self.blackboard.vlm_plan = plan_data
                    return py_trees.common.Status.SUCCESS
                except Exception as e:
                    self.node.get_logger().error(f"Failed to parse VLM plan: {e}")
                    return py_trees.common.Status.FAILURE
            else:
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING
