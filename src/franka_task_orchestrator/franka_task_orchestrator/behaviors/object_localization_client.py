#!/usr/bin/env python3

"""
================================================================================
Author: Falco Robotics
Code Description:
[ROLE]: ACTION CLIENT (Calls: /detect_object)

This Behavior Tree leaf node interfaces with the YOLO Object Localization Server.
It reads the requested 'target_label' dynamically from the BT Blackboard (which 
was populated by the preceding VLM node), transmits the detection goal, and 
writes the resulting 3D PoseStamped back to the Blackboard so that the execution 
layer (MoveIt Task Constructor) can grasp it.

Pipeline: BT Orchestration -> Perception Client
================================================================================
"""

import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import DetectObject

class ObjectLocalizationClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="YOLO Localization", action_name="/detect_object"):
        super().__init__(name=name)
        self.action_name = action_name
        
        self.node = None
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="target_label", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_pose", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError:
            raise KeyError("The ROS 2 node was not passed to the setup() method by the py_trees runner.")

        self.action_client = ActionClient(self.node, DetectObject, self.action_name)
        self.node.get_logger().info(f"[{self.name}] Waiting for Action Server {self.action_name}...")
        self.action_client.wait_for_server(timeout_sec=15.0)

    def initialise(self):
        target_label = "none"
        try:
            target_label = self.blackboard.target_label
        except AttributeError:
            self.node.get_logger().error(f"[{self.name}] target_label not found in Blackboard!")
        
        goal_msg = DetectObject.Goal()
        goal_msg.object_name = target_label

        self.node.get_logger().info(f"[{self.name}] Requesting YOLO localization for: '{target_label}'")
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
                self.node.get_logger().error(f"[{self.name}] Goal rejected by YOLO Action Server.")
                return py_trees.common.Status.FAILURE
            self.node.get_logger().info(f"[{self.name}] YOLO Goal Accepted. Processing frames...")
            self.get_result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if self.get_result_future and not self.get_result_future.done():
            return py_trees.common.Status.RUNNING

        if self.get_result_future and self.get_result_future.done():
            result = self.get_result_future.result().result
            if result.success:
                self.node.get_logger().info(f"[{self.name}] Localization Complete. 3D Pose saved to Blackboard.")
                self.blackboard.target_pose = result.target_pose
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.send_goal_future = None
        self.get_result_future = None
