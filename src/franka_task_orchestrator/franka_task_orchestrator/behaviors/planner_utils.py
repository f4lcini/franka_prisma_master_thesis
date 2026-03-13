#!/usr/bin/env python3

import py_trees
import operator

class DynamicActionIterator(py_trees.behaviour.Behaviour):
    """
    Control node that iterates through a list of actions stored in the blackboard.
    It pops the first action from the list and populates the blackboard with its
    parameters for the execution nodes to use.
    """
    def __init__(self, name="Dynamic Action Iterator"):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="vlm_plan", access=py_trees.common.Access.READ)
        # Parameters for execution nodes
        self.blackboard.register_key(key="target_label", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="active_arm", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="active_action", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="grasp_type", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_location", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')

    def initialise(self):
        self.node.get_logger().info(f"[{self.name}] Initializing sequence iterator...")

    def update(self):
        try:
            plan = self.blackboard.vlm_plan
        except AttributeError:
            self.node.get_logger().error(f"[{self.name}] vlm_plan not found on blackboard!")
            return py_trees.common.Status.FAILURE

        if plan is None or len(plan) == 0:
            self.node.get_logger().info(f"[{self.name}] No more actions in the plan.")
            return py_trees.common.Status.SUCCESS

        # Take the next action from the sequence
        # Note: In a real BT, you might want to 'pop' or just keep an index.
        # Since we want to repeat the execution branch for each action, 
        # a simple way is to 'pop' the action once it's picked for execution.
        current_action_task = plan[0]
        
        # Mapping Skill fields to Blackboard keys
        action_type = current_action_task.get("action")
        self.blackboard.active_action = action_type
        self.blackboard.target_label = current_action_task.get("target_name", "none")
        self.blackboard.active_arm = current_action_task.get("arm", "any")
        self.blackboard.grasp_type = current_action_task.get("grasp_type", "top")
        self.blackboard.target_location = current_action_task.get("target_location", "none")

        self.node.get_logger().info(f"[{self.name}] Next Action: {action_type} on {self.blackboard.target_label} using {self.blackboard.active_arm}")
        
        # We return SUCCESS to indicate we have prepared the blackboard.
        # The parent sequence will then move on to the actual execution nodes.
        return py_trees.common.Status.SUCCESS

class PlanPopper(py_trees.behaviour.Behaviour):
    """Simple behavior to remove the processed action from the plan."""
    def __init__(self, name="Plan Popper"):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="vlm_plan", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')

    def update(self):
        plan = self.blackboard.vlm_plan
        if plan and len(plan) > 0:
            removed = plan.pop(0)
            self.node.get_logger().info(f"[{self.name}] Action '{removed['action']}' completed. Popping from list.")
            self.blackboard.vlm_plan = plan
        return py_trees.common.Status.SUCCESS
