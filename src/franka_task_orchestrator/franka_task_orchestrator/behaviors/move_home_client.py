import py_trees
import time

class MoveHomeClient(py_trees.behaviour.Behaviour):
    """
    Move Home Client (Fallback / Skeleton).
    Moves the specified arm back to its resting position.
    """
    def __init__(self, name: str):
        super(MoveHomeClient, self).__init__(name=name)
        self.start_time = None
        self.duration = 3.0 # 3 seconds of "simulated moving"

    def setup(self, **kwargs):
        return

    def initialise(self):
        self.start_time = time.time()
        self.logger.info(f"[{self.name}] SIMULATED MOVE_HOME: Starting {self.duration}s execution...")

    def update(self):
        elapsed = time.time() - self.start_time
        
        if elapsed < self.duration:
            self.logger.debug(f"[{self.name}] SIMULATING GOING HOME... ({elapsed:.1f}s)")
            return py_trees.common.Status.RUNNING
        
        self.logger.info(f"[{self.name}] SIMULATED MOVE_HOME COMPLETE. Returning SUCCESS.")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.start_time = None
        self.logger.debug(f"{self.name}: terminate({new_status})")
