#!/usr/bin/env python3

import py_trees
import operator

class PlanSplitter(py_trees.behaviour.Behaviour):
    """
    Takes the global 'vlm_plan' and splits it into 'left_arm_plan' and 'right_arm_plan'.
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
        
        left_plan = [a for a in plan if a.get("arm", "").lower().startswith("left")]
        right_plan = [a for a in plan if a.get("arm", "").lower().startswith("right")]
        
        self.blackboard.left_arm_plan = left_plan
        self.blackboard.right_arm_plan = right_plan
        
        self.node.get_logger().info(f"[{self.name}] Plan Split: Left({len(left_plan)}) | Right({len(right_plan)})")
        return py_trees.common.Status.SUCCESS

class DynamicActionIterator(py_trees.behaviour.Behaviour):
    """
    Iterates through an arm-specific plan and populates namespaced blackboard keys.
    """
    def __init__(self, name="Action Iterator", plan_key="left_arm_plan", prefix="left_"):
        super().__init__(name=name)
        self.plan_key = plan_key
        self.prefix = prefix
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=plan_key, access=py_trees.common.Access.READ)
        
        # Namespaced Parameters
        for key in ["target_name", "active_arm", "active_action", "grasp_type", "target_location"]:
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
        setattr(self.blackboard, f"{self.prefix}active_arm", current.get("arm", "any"))
        setattr(self.blackboard, f"{self.prefix}grasp_type", current.get("grasp_type", "top"))
        setattr(self.blackboard, f"{self.prefix}target_location", current.get("target_location", "none"))

        self.node.get_logger().info(f"[{self.name}] Next '{self.prefix}' Action: {current.get('action')}")
        return py_trees.common.Status.SUCCESS

class PlanPopper(py_trees.behaviour.Behaviour):
    """Simple behavior to remove the processed action from the plan."""
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
                removed = plan.pop(0)
                self.node.get_logger().info(f"[{self.name}] Action '{removed['action']}' completed. Popping from '{self.plan_key}'.")
                setattr(self.blackboard, self.plan_key, plan)
        except Exception as e:
            self.node.get_logger().error(f"[{self.name}] Popper error: {e}")
            
        return py_trees.common.Status.SUCCESS
