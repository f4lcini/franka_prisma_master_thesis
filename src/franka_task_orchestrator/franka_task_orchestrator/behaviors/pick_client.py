import py_trees
import time

class MtcPickActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Execute MTC Pick", action_name="/mtc_pick_object"):
        super().__init__(name=name)
        self.action_name = action_name
        
        # Mock variables
        self.start_time = None
        self.duration = 4.0 # 4 seconds of "simulated moving"

        # Sincronizzazione Blackboard
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="target_pose", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """Mock setup: always succeeds without needing a server."""
        return

    def initialise(self):
        """Reset the timer at the start of the behavior."""
        self.start_time = time.time()
        self.logger.info(f"[{self.name}] SIMULATED PICK: Starting 4s execution...")

    def update(self):
        """State machine: RUNNING for 4s, then SUCCESS."""
        elapsed = time.time() - self.start_time
        
        if elapsed < self.duration:
            self.logger.debug(f"[{self.name}] SIMULATING MOVE... ({elapsed:.1f}s)")
            return py_trees.common.Status.RUNNING
        
        self.logger.info(f"[{self.name}] SIMULATED PICK COMPLETE. Returning SUCCESS.")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.start_time = None