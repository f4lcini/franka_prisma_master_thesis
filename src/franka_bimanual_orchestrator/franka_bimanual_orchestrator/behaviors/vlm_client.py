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
import json
from rclpy.action import ActionClient
from franka_custom_interfaces.action import VlmQuery
from franka_bimanual_skills.skills_repertoire import TaskPlan

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
        # We now store the entire sequence plan for the iterator to process
        self.blackboard.register_key(key="vlm_plan", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError:
            raise KeyError("The ROS 2 node was not passed to the setup() method by the py_trees runner.")

        self.action_client = ActionClient(self.node, VlmQuery, self.action_name)
        self.node.get_logger().info(f"[{self.name}] Waiting for VLM Action Server {self.action_name}...")
        self.action_client.wait_for_server(timeout_sec=15.0)

    def initialise(self):
        goal_msg = VlmQuery.Goal()
        goal_msg.task_description = self.task_description
        # Clear previous plan
        self.blackboard.vlm_plan = None
        
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
            if result.success:
                try:
                    # Validate and Parse using Pydantic
                    plan = TaskPlan.parse_raw(result.vlm_plan_json)
                    
                    self.node.get_logger().info(f"[{self.name}] VLM Success (Validated)!\nReasoning: {plan.reasoning}")
                    self.node.get_logger().info(f"[{self.name}] Plan Sequence: {[a.action for a in plan.sequence]}")
                    
                    if not plan.sequence:
                        self.node.get_logger().warning(f"[{self.name}] ⚠️ RECEIVED EMPTY PLAN SEQUENCE!")
                    
                    # Store sequence in blackboard (as list of dicts for the existing iterator)
                    self.blackboard.vlm_plan = [a.dict() for a in plan.sequence]
                    return py_trees.common.Status.SUCCESS
                except Exception as e:
                    self.node.get_logger().error(f"[{self.name}] Pydantic Validation Failed: {e}")
                    return py_trees.common.Status.FAILURE
            else:
                self.node.get_logger().error(f"[{self.name}] VLM Server returned failure: {result.message}")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.send_goal_future = None
        self.get_result_future = None
