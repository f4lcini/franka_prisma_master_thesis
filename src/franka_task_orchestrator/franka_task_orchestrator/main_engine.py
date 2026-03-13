#!/usr/bin/env python3

"""
================================================================================
Author: Falco Robotics 
Code Description: 
This is the central execution framework (Main Engine) employing a Behavior Tree 
(BT) to orchestrate tasks across the robotic cell. It uses py_trees and py_trees_ros 
to build a Directed Acyclic Graph (DAG) that coordinates High-Level perception (VLM) 
with Low-Level execution (MoveIt Task Constructor or direct Cartesians).

Pipeline: Task Orchestration & Mission Control

Implementation Steps Summary:
- NODE & DEPENDENCY SETUP (Steps 1-2): Import the py_trees architecture and internal behavior modules (MockPerception, PickClient).
- TREE CONSTRUCTION (Steps 3-4): Create a root Sequence with Memory to build a strict execution loop that saves completed behavior states.
- BEHAVIOR INSTANTIATION (Step 5): Initialize the specific Behavior Leaf nodes (e.g., Simulate_Vision_Layer, MTC_Hardware_Execution).
- TOPOLOGY LINKING (Step 6): Attach the initialized behaviors as children to the root DAG Sequence.
- ROS 2 WRAPPER INITIALIZATION (Steps 7-8): Launch the main function and securely wrap the py_trees object inside a py_trees_ros BehaviourTree node.
- SYSTEM HANDSHAKE (Steps 9-10): Call tree.setup() with a rigorous Timeout to ensure all external Action Servers (MTC, VLM) are reachable before starting.
- ASYNCHRONOUS TICK LOOP (Step 11): Run the node using tick_tock, triggering cyclic tree executions (e.g., 2 Hz) until completion or SIGINT.
- GRACEFUL SHUTDOWN (Step 12): Systematically shutdown the tree and drop rclpy loops when the process terminates.
================================================================================
"""

import rclpy
import py_trees
import py_trees_ros
import sys
import operator

from franka_task_orchestrator.behaviors.vlm_client import VlmActionClient
from franka_task_orchestrator.behaviors.object_localization_client import ObjectLocalizationClient
from franka_task_orchestrator.behaviors.pick_client import MtcPickActionClient
from franka_task_orchestrator.behaviors.planner_utils import DynamicActionIterator, PlanPopper

def create_tree(task_description="Default Task"):
    """Constructs a dynamic Behavior Tree for Dual-Arm Orchestration."""
    
    # Root: Orchestrates Planning then Execution
    root = py_trees.composites.Sequence(name="VLM_Orchestrator", memory=True)

    # 1. PLANNING PHASE
    vlm_planner = VlmActionClient(name="Gemini_Planner", task_description=task_description)
    
    # 2. EXECUTION PHASE (LOOP)
    # We use a Repeat decorator. The loop will continue as long as the child returns SUCCESS.
    # Inside, we check if the plan is empty to break the loop.
    execution_step = py_trees.composites.Sequence(name="Execution_Step", memory=True)
    
    execution_loop = py_trees.decorators.Repeat(
        child=execution_step,
        num_success=10, # Max steps as a safety limit
        name="Plan_Execution_Loop"
    )
    
    # --- Check if Plan is finished ---
    # This node returns FAILURE if the plan is empty, which stops the Repeat decorator
    plan_not_empty = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Plan_Not_Empty?",
        check=py_trees.common.ComparisonExpression(
            variable="vlm_plan",
            value=[],
            operator=operator.ne
        )
    )

    # A. Iterator: Picks the next action
    iterator = DynamicActionIterator(name="Fetch_Next_Action")
    
    # B. Action Dispatcher
    dispatcher = py_trees.composites.Selector(name="Skill_Dispatcher", memory=False)
    
    # Skill: FIND_OBJECT
    search_sequence = py_trees.composites.Sequence(name="Search_Sequence", memory=True)
    is_search = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Is_Search?",
        check=py_trees.common.ComparisonExpression(
            variable="active_action",
            value="FIND_OBJECT",
            operator=operator.eq
        )
    )
    yolo_client = ObjectLocalizationClient(name="YOLO_Localization")
    search_sequence.add_children([is_search, yolo_client])
    
    # Skill: PICK
    pick_sequence = py_trees.composites.Sequence(name="Pick_Sequence", memory=True)
    is_pick = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Is_Pick?",
        check=py_trees.common.ComparisonExpression(
            variable="active_action",
            value="PICK",
            operator=operator.eq
        )
    )
    pick_client = MtcPickActionClient(name="MTC_Pick_Execution")
    pick_sequence.add_children([is_pick, pick_client])

    dispatcher.add_children([search_sequence, pick_sequence])
    
    # C. Housekeeping
    popper = PlanPopper(name="Complete_Step")
    
    execution_step.add_children([plan_not_empty, iterator, dispatcher, popper])
    
    root.add_children([vlm_planner, execution_loop])
    
    return root

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Franka Task Orchestrator - Dynamic Engine")
    parser.add_argument("task", type=str, nargs="?", default="Pick up the red cube", help="Task for the VLM")
    
    args_without_ros = rclpy.utilities.remove_ros_args(args=sys.argv)
    parsed_args = parser.parse_args(args_without_ros[1:])

    rclpy.init(args=sys.argv)
    
    root = create_tree(task_description=parsed_args.task)
    
    # Wrap in py_trees_ros
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    
    try:
        # 15 seconds timeout for action servers
        tree.setup(node_name="task_orchestrator_engine", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        print(f"Server Timeout: {e}. Check VLM and Perception/MTC nodes.")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    print("\n--- Dynamic Orchestrator Initialized ---")
    print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
    
    try:
        tree.tick_tock(period_ms=500)
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()