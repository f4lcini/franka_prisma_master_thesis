#!/usr/bin/env python3

import py_trees
import rclpy
import time
import operator

class WaitActionClient(py_trees.behaviour.Behaviour):
    """
    Simulates a wait skill for bimanual coordination.
    It can wait for a fixed duration OR for a blackboard flag 'handover_ready'.
    """
    def __init__(self, name="Wait for Handover", prefix="left_"):
        super().__init__(name=name)
        self.prefix = prefix
        self.start_time = None
        self.wait_duration = 5.0 # Fallback small wait
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        # Global keys for handover synchronization
        self.blackboard.register_key(key="handover_ready", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="handover_starting", access=py_trees.common.Access.READ)
        
    def initialise(self):
        self.logger.info(f"[{self.name}] Synchronizing... Waiting for 'handover_ready' flag.")
        self.start_time = time.time()

    def update(self):
        # 1. Check Blackboard Flag (Early Signal for smoother overlap)
        try:
            if getattr(self.blackboard, "handover_starting", False) == True:
                self.logger.info(f"[{self.name}] Handover Starting detected. Proceeding with Anticipatory Pick approach.")
                return py_trees.common.Status.SUCCESS
        except AttributeError:
            pass

        # 2. Status Output
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.start_time = None
