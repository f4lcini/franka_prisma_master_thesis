#!/usr/bin/env python3
import py_trees

class MissionAbort(py_trees.behaviour.Behaviour):
    """Sets a global blackboard flag to abort the entire mission."""
    def __init__(self, name="Mission Abort"):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="mission_aborted", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="abort_message", access=py_trees.common.Access.WRITE)

    def update(self):
        self.blackboard.mission_aborted = True
        self.blackboard.abort_message = f"Mission aborted by {self.name}"
        self.logger.error(f"🛑 CRITICAL: Mission Abort signal set by {self.name}")
        return py_trees.common.Status.SUCCESS

class ReplanTrigger(py_trees.behaviour.Behaviour):
    """Clears all plans to trigger a fresh re-planning cycle from Gemini."""
    def __init__(self, name="Replan Trigger"):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="vlm_plan", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="left_arm_plan", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="right_arm_plan", access=py_trees.common.Access.WRITE)

    def update(self):
        self.blackboard.vlm_plan = None
        self.blackboard.left_arm_plan = []
        self.blackboard.right_arm_plan = []
        self.logger.warning(f"🔄 REPLAN: All plans cleared. Ready for next VLM cycle.")
        # Return FAILURE to break the current execution parallel and trigger a loop restart
        return py_trees.common.Status.FAILURE

class MissionFail(py_trees.behaviour.Behaviour):
    """Signals that the mission has failed and cannot be recovered (Terminal State)."""
    def __init__(self, name="Mission Fail"):
        super().__init__(name=name)

    def update(self):
        self.logger.error("💀 TERMINAL FAILURE: Mission failed (likely IK or Hardware error).")
        import os
        import signal
        os.kill(os.getpid(), signal.SIGINT)
        return py_trees.common.Status.FAILURE

class MissionExit(py_trees.behaviour.Behaviour):
    """Signals that the mission is successfully finished and shuts down the process."""
    def __init__(self, name="Mission Exit"):
        super().__init__(name=name)

    def update(self):
        self.logger.info("🏁 MISSION COMPLETE: Object delivered. Shutting down process...")
        import os
        import signal
        os.kill(os.getpid(), signal.SIGINT)
        return py_trees.common.Status.SUCCESS
