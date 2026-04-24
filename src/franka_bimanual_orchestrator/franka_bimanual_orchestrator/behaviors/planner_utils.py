import py_trees

class PlanSplitter(py_trees.behaviour.Behaviour):
    """
    Takes the TaskPlan from 'vlm_plan' and populates 'left_arm_plan' and 'right_arm_plan'.
    """
    def __init__(self, name="Plan Splitter"):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="vlm_plan", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="left_arm_plan", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="right_arm_plan", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')

    def update(self):
        plan = self.blackboard.vlm_plan
        if plan is None:
            return py_trees.common.Status.FAILURE
        
        # New TaskPlan structure has explicit lists
        self.blackboard.left_arm_plan = plan.get("left_arm_sequence", [])
        self.blackboard.right_arm_plan = plan.get("right_arm_sequence", [])
        
        return py_trees.common.Status.SUCCESS

class DynamicActionIterator(py_trees.behaviour.Behaviour):
    def __init__(self, name="Action Iterator", plan_key="left_arm_plan", prefix="left_"):
        super().__init__(name=name)
        self.plan_key = plan_key
        self.prefix = prefix
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=plan_key, access=py_trees.common.Access.READ)
        
        # Extended parameters for new skills
        keys = ["target_name", "active_arm", "active_action", "grasp_type", "target_location", "target_pose_name"]
        for key in keys:
            self.blackboard.register_key(key=f"{prefix}{key}", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')

    def update(self):
        try:
            plan = getattr(self.blackboard, self.plan_key)
        except AttributeError:
            return py_trees.common.Status.FAILURE

        if not plan:
            return py_trees.common.Status.SUCCESS

        current = plan[0]
        setattr(self.blackboard, f"{self.prefix}active_action", current.get("action"))
        setattr(self.blackboard, f"{self.prefix}target_name", current.get("target_name", "none"))
        
        # Support both nested 'arm' object and flat string for flexibility
        arm_data = current.get("arm", "any")
        if isinstance(arm_data, dict):
            arm_data = arm_data.get("arm", "any")
        setattr(self.blackboard, f"{self.prefix}active_arm", arm_data)
        
        setattr(self.blackboard, f"{self.prefix}grasp_type", current.get("grasp_type", "top"))
        setattr(self.blackboard, f"{self.prefix}target_location", current.get("target_location", "none"))
        setattr(self.blackboard, f"{self.prefix}target_pose_name", current.get("target_pose_name", "ready"))

        return py_trees.common.Status.SUCCESS

class PlanPopper(py_trees.behaviour.Behaviour):
    def __init__(self, name="Plan Popper", plan_key="left_arm_plan"):
        super().__init__(name=name)
        self.plan_key = plan_key
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=plan_key, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')

    def update(self):
        try:
            plan = getattr(self.blackboard, self.plan_key)
            if plan and len(plan) > 0:
                plan.pop(0)
                setattr(self.blackboard, self.plan_key, plan)
        except Exception:
            pass
        return py_trees.common.Status.SUCCESS
