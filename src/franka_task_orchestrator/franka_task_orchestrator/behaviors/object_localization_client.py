#!/usr/bin/env python3

import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import DetectObject

class ObjectLocalizationClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="YOLO Localization", action_name="/detect_object", prefix="left_"):
        super().__init__(name=name)
        self.action_name = action_name
        self.prefix = prefix
        self.node = None
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=f"{prefix}target_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"{prefix}target_pose", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError:
            return False

        self.action_client = ActionClient(self.node, DetectObject, self.action_name)
        # Make it non-blocking for No-YOLO testing
        if not self.action_client.wait_for_server(timeout_sec=0.5):
             self.node.get_logger().warning(f"[{self.name}] Action Server {self.action_name} not found.")
        return True

    def initialise(self):
        target_name = "none"
        if hasattr(self.blackboard, f"{self.prefix}target_name"):
            target_name = getattr(self.blackboard, f"{self.prefix}target_name")
        
        goal_msg = DetectObject.Goal()
        goal_msg.object_name = target_name

        self.node.get_logger().info(f"[{self.name}] Requesting YOLO localization for: '{target_name}'")
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.get_result_future = None

    def update(self):
        if not self.send_goal_future:
            return py_trees.common.Status.FAILURE
        if self.send_goal_future and not self.send_goal_future.done():
            return py_trees.common.Status.RUNNING
        if self.send_goal_future and self.send_goal_future.done() and not self.get_result_future:
            goal_handle = self.send_goal_future.result()
            if not goal_handle.accepted:
                return py_trees.common.Status.FAILURE
            self.get_result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING
        if self.get_result_future and not self.get_result_future.done():
            return py_trees.common.Status.RUNNING
        if self.get_result_future and self.get_result_future.done():
            result = self.get_result_future.result().result
            if result.success:
                setattr(self.blackboard, f"{self.prefix}target_pose", result.target_pose)
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.send_goal_future = None
        self.get_result_future = None
