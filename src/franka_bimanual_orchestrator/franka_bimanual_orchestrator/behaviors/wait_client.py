import py_trees
import rclpy
import time
from std_msgs.msg import String

class WaitActionClient(py_trees.behaviour.Behaviour):
    """
    Simulates a wait skill for bimanual coordination.
    It can wait for a fixed duration OR for a ROS 2 Lock flag.
    """
    def __init__(self, name="Wait for Handover", prefix="left_", duration=None):
        super().__init__(name=name)
        self.prefix = prefix
        self.duration = duration
        self.start_time = None
        self.ros_lock_signal = None
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="handover_ready", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="handover_starting", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="mission_type", access=py_trees.common.Access.READ)
        
    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.lock_sub = self.node.create_subscription(
            String, "/franka/shared_zone/lock", self._lock_callback, 10
        )
        return True

    def _lock_callback(self, msg):
        self.ros_lock_signal = msg.data

    def initialise(self):
        self.ros_lock_signal = None
        self.start_time = time.time()
        if self.duration:
            self.logger.info(f"⏳ [{self.name}] Starting TIMED WAIT for {self.duration} seconds...")
        else:
            self.logger.info(f"[{self.name}] Synchronizing via ROS 2 Lock...")

    def update(self):
        # --- 1. TIMED WAIT LOGIC (If duration is provided) ---
        if self.duration is not None:
            elapsed = time.time() - self.start_time
            if elapsed >= self.duration:
                self.logger.info(f"✅ [{self.name}] Timed wait COMPLETED.")
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING

        # --- 2. SIGNAL WAIT LOGIC (Original logic) ---
        mission_type = getattr(self.blackboard, "mission_type", "SIMPLE")
        if mission_type != "HANDOVER":
            return py_trees.common.Status.SUCCESS

        partner_prefix = "RIGHT" if self.prefix == "left_" else "LEFT"
        expected_signal = f"{partner_prefix}ARM_RELEASED"

        if self.ros_lock_signal == expected_signal or self.blackboard.handover_ready:
             self.logger.info(f"🔓 [{self.name}] Proceeding (Signal or Blackboard).")
             return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.start_time = None
