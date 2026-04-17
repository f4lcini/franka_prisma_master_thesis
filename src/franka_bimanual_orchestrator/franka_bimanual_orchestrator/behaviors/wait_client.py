#!/usr/bin/env python3

import py_trees
import rclpy
import time
from std_msgs.msg import Bool

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

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        if self.node:
            self._sub = self.node.create_subscription(
                Bool, '/handover_ready', self._ready_cb, 10)
        return True

    def _ready_cb(self, msg: Bool):
        if msg.data:
            self._ready = True
            self.logger.info(f"[{self.name}] /handover_ready received! Proceeding.")

    def initialise(self):
        self.logger.info(f"[{self.name}] Waiting for /handover_ready (timeout={self.timeout_sec}s)...")
        self.start_time = time.time()
        self._ready = False   # reset in case this runs more than once

    def update(self):
        if self._ready:
            return py_trees.common.Status.SUCCESS

        elapsed = time.time() - self.start_time
        if elapsed >= self.timeout_sec:
            self.logger.warning(f"[{self.name}] Timeout after {self.timeout_sec}s — proceeding anyway.")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.start_time = None
        self._ready = False
