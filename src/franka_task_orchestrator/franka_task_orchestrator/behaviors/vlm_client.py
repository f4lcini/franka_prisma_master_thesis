#!/usr/bin/env python3

"""
================================================================================
Author: Falco Robotics
Code Description:
[ROLE]: ACTION CLIENT (Calls: /vlm_query)

This Behavior Tree leaf node queries the VLM Server. It sends the natural 
language task description to Gemini and, upon success, writes the structured 
decision (target_label, action_choice, selected_arm) to the BT Blackboard 
so that subsequent nodes (like YOLO Localization) can utilize them without 
direct coupling.

Pipeline: BT Orchestration -> Reasoning Client
================================================================================
"""

import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import VlmQuery

class VlmActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="VLM Query", action_name="/vlm_query", task_description="Default description of the query"):
        super().__init__(name=name)
        self.action_name = action_name
        self.task_description = task_description
        
        self.node = None
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="target_label", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="action_choice", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="selected_arm", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="handover_height_z", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError:
            raise KeyError("The ROS 2 node was not passed to the setup() method by the py_trees runner.")

        self.action_client = ActionClient(self.node, VlmQuery, self.action_name)
        self.node.get_logger().info(f"[{self.name}] Waiting for Action Server {self.action_name}...")
        self.action_client.wait_for_server(timeout_sec=15.0)

    def initialise(self):
        goal_msg = VlmQuery.Goal()
        goal_msg.task_description = self.task_description
        self.node.get_logger().info(f"[{self.name}] Transmitting VLM Task: '{self.task_description}'")
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.get_result_future = None

    def update(self):
        if self.send_goal_future and not self.send_goal_future.done():
            return py_trees.common.Status.RUNNING

        if self.send_goal_future and self.send_goal_future.done() and not self.get_result_future:
            goal_handle = self.send_goal_future.result()
            if not goal_handle.accepted:
                self.node.get_logger().error(f"[{self.name}] Goal rejected by VLM Action Server.")
                return py_trees.common.Status.FAILURE
            self.node.get_logger().info(f"[{self.name}] VLM Goal Accepted. Awaiting Gemini reasoning...")
            self.get_result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if self.get_result_future and not self.get_result_future.done():
            return py_trees.common.Status.RUNNING

        if self.get_result_future and self.get_result_future.done():
            result = self.get_result_future.result().result
            if result.success and result.target_label.lower() != "none":
                self.node.get_logger().info(f"[{self.name}] VLM Success! Found target: '{result.target_label}'")
                self.blackboard.target_label = result.target_label
                self.blackboard.action_choice = result.action_choice
                self.blackboard.selected_arm = result.arm_selection
                self.blackboard.handover_height_z = result.handover_height_z
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.send_goal_future = None
        self.get_result_future = None
