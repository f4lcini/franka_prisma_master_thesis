import py_trees
import rclpy
import time
from std_msgs.msg import String

class WaitActionClient(py_trees.behaviour.Behaviour):
    """
    Simulates a wait skill for bimanual coordination.
    It can wait for a fixed duration OR for a ROS 2 Lock flag.
    """
    def __init__(self, name="Wait for Handover", prefix="left_"):
        super().__init__(name=name)
        self.prefix = prefix
        self.start_time = None
        self.wait_duration = 5.0 # Fallback small wait
        self.ros_lock_signal = None
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        # Global keys for handover synchronization
        self.blackboard.register_key(key="handover_ready", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="handover_starting", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="mission_type", access=py_trees.common.Access.READ)
        
    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        # --- SHARED ZONE LOCK SUBSCRIBER ---
        self.lock_sub = self.node.create_subscription(
            String, "/franka/shared_zone/lock", self._lock_callback, 10
        )
        return True

    def _lock_callback(self, msg):
        self.ros_lock_signal = msg.data
        self.logger.info(f"📩 [{self.name}] ROS 2 LOCK RECEIVED: {self.ros_lock_signal}")

    def initialise(self):
        self.logger.info(f"[{self.name}] Synchronizing via ROS 2 Lock...")
        self.ros_lock_signal = None # Reset for new wait

    def update(self):
        # 1. Skip if not a coordinated mission
        mission_type = getattr(self.blackboard, "mission_type", "SIMPLE")
        if mission_type != "HANDOVER":
            self.logger.info(f"[{self.name}] SIMPLE mission detected. Skipping wait.")
            return py_trees.common.Status.SUCCESS

        # 2. Check ROS 2 Lock Signal
        # Partner arm (usually Right) sends RELEASED signal
        partner_prefix = "RIGHT" if self.prefix == "left_" else "LEFT"
        expected_signal = f"{partner_prefix}ARM_RELEASED"

        if self.ros_lock_signal == expected_signal:
             self.logger.info(f"🔓 [{self.name}] ROS 2 LOCK RELEASED by partner! Proceeding.")
             return py_trees.common.Status.SUCCESS

        # 3. Fallback: Still check blackboard (Hybrid approach)
        if self.blackboard.handover_ready:
             self.logger.info(f"🚩 [{self.name}] Blackboard fallback: ready detected. Proceeding.")
             return py_trees.common.Status.SUCCESS

        # Periodically log that we are still waiting
        self.logger.info(f"[{self.name}] Still waiting for {expected_signal} on /franka/shared_zone/lock...")
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.start_time = None
