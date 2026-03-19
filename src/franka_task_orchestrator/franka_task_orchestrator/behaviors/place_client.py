import py_trees
import time

class MtcPlaceActionClient(py_trees.behaviour.Behaviour):
    """
    Place Client for MTC (Fallback / Skeleton).
    Retrieves the arm and target_location from the current active action parameters.
    """
    def __init__(self, name: str):
        super(MtcPlaceActionClient, self).__init__(name=name)
        self.start_time = None
        self.duration = 4.0 # 4 seconds of "simulated moving"

    def setup(self, **kwargs):
        return

    def initialise(self):
        self.start_time = time.time()
        self.logger.info(f"[{self.name}] SIMULATED PLACE: Starting {self.duration}s execution...")

    def update(self):
        elapsed = time.time() - self.start_time
        
        if elapsed < self.duration:
            self.logger.debug(f"[{self.name}] SIMULATING PLACE MOVE... ({elapsed:.1f}s)")
            return py_trees.common.Status.RUNNING
        
        self.logger.info(f"[{self.name}] SIMULATED PLACE COMPLETE. Returning SUCCESS.")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.start_time = None
        self.logger.debug(f"{self.name}: terminate({new_status})")
