#!/usr/bin/env python3

import py_trees
import rclpy
import time
from std_msgs.msg import Bool
from rclpy.action import ActionClient
from franka_custom_interfaces.action import MoveHome

class WaitActionClient(py_trees.behaviour.Behaviour):
    """
    Waits for a '/handover_ready' ROS topic signal (published by the Place server)
    with a configurable timeout fallback so execution never blocks permanently.
    """
    def __init__(self, name="Wait for Handover", prefix="left_"):
        super().__init__(name=name)
        self.prefix = prefix
        self.start_time = None
        self.timeout_sec = 30.0   # max wait time
        self._ready = False
        self._sub = None
        self.node = None
        
        # Action Client for Proactive Move
        self.move_client = None
        self.move_goal_future = None
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=f"{prefix}active_arm", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        if self.node:
            self._sub = self.node.create_subscription(
                Bool, '/handover_ready', self._ready_cb, 10)
            self.move_client = ActionClient(self.node, MoveHome, "/move_home")
        return True

    def _ready_cb(self, msg: Bool):
        if msg.data:
            self._ready = True
            self.logger.info(f"[{self.name}] /handover_ready received! Proceeding.")

    def initialise(self):
        self.logger.info(f"[{self.name}] Waiting for '{self.prefix[:-1]}' arm handover signal...")
        self.start_time = time.time()
        self._ready = False

    def update(self):
        if self._ready:
            return py_trees.common.Status.SUCCESS

        elapsed = time.time() - self.start_time
        if elapsed > 60.0:
            self.logger.error(f"[{self.name}] Handover wait TIMEOUT (60s)")
            return py_trees.common.Status.FAILURE
            
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.start_time = None
        self._ready = False
