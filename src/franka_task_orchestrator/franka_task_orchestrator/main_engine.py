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
from franka_task_orchestrator.behaviors.place_client import MtcPlaceActionClient
from franka_task_orchestrator.behaviors.move_home_client import MoveHomeClient
from franka_task_orchestrator.behaviors.planner_utils import DynamicActionIterator, PlanPopper

def create_tree(task_description="Default Task"):
    """Constructs a dynamic Behavior Tree for Dual-Arm Orchestration."""
    
    # --- Top Level Logic: One-Shot Execution ---
    # This selector ensures that once the task is finished, we don't restart it.
    root = py_trees.composites.Selector(name="Root_Guard", memory=True)

    # A. Check if mission is already accomplished
    mission_done = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Mission_Completed?",
        check=py_trees.common.ComparisonExpression(
            variable="mission_completed",
            value=True,
            operator=operator.eq
        )
    )

    # B. Main Orchestrator (The original logic)
    main_sequence = py_trees.composites.Sequence(name="VLM_Orchestrator", memory=True)

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
    
    # === SKILL REPERTOIRE (REGISTRY) ===
    # Map the action string from VLM to the corresponding Action Client Behavior
    available_skills = {
        "FIND_OBJECT": ObjectLocalizationClient(name="YOLO_Localization"),
        "PICK": MtcPickActionClient(name="MTC_Pick_Execution"),
        "PLACE": MtcPlaceActionClient(name="MTC_Place_Execution"),
        "MOVE_HOME": MoveHomeClient(name="Move_Home_Execution")
    }

    # Dynamically build the dispatcher routing based on the available skills
    for skill_name, client_node in available_skills.items():
        skill_sequence = py_trees.composites.Sequence(name=f"{skill_name}_Sequence", memory=True)
        is_active = py_trees.behaviours.CheckBlackboardVariableValue(
            name=f"Is_{skill_name}?",
            check=py_trees.common.ComparisonExpression(
                variable="active_action",
                value=skill_name,
                operator=operator.eq
            )
        )
        skill_sequence.add_children([is_active, client_node])
        dispatcher.add_child(skill_sequence)
    
    # C. Housekeeping
    popper = PlanPopper(name="Complete_Step")
    
    execution_step.add_children([plan_not_empty, iterator, dispatcher, popper])
    
    # --- RECOVERY GUARD (UNIVERSAL FALLBACK) ---
    # This selector acts as a safety net. If ANY skill in the execution_loop 
    # returns FAILURE (e.g. YOLO can't find object), the loop fails, and 
    # control falls to the second child (Abort_Mission_Sequence).
    recovery_guard = py_trees.composites.Selector(name="Execution_Recovery_Guard", memory=False)
    
    abort_sequence = py_trees.composites.Sequence(name="Abort_Mission_Sequence", memory=True)
    
    # If we fall back here, we mark mission completed=True so the Root Guard doesn't endlessly restart Gemini
    mark_failed_stop_loop = py_trees.behaviours.SetBlackboardVariable(
        name="Mark_Mission_Failed_Stop_VLM",
        variable_name="mission_completed",
        variable_value=True, 
        overwrite=True
    )
    
    # EMERGENZA FISICA: Comando il braccio che ha fallito (ancora in blackboard active_action["arm"]) di tornare immediatamente a Home
    emergency_move_home = MoveHomeClient(name="Emergency_Move_Home")
    
    abort_sequence.add_children([emergency_move_home, mark_failed_stop_loop])
    
    # The guard first tries the normal loop, if it breaks/fails, it runs the abort sequence
    recovery_guard.add_children([execution_loop, abort_sequence])
    
    # D. Finalize Mission
    # This node sets a flag on the blackboard to signal normal completion
    mark_done = py_trees.behaviours.SetBlackboardVariable(
        name="Mark_Mission_Done",
        variable_name="mission_completed",
        variable_value=True,
        overwrite=True
    )

    main_sequence.add_children([vlm_planner, recovery_guard, mark_done])
    
    root.add_children([mission_done, main_sequence])
    
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
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    
    # We disable the loud ASCII terminal printouts to keep the console clean.
    # To view the tree dynamically, use the ROS 2 py-tree-viewer GUI on the side.
    tree.visitors.append(py_trees.visitors.DisplaySnapshotVisitor())
    
    try:
        # 15 seconds timeout for action servers
        tree.setup(node_name="task_orchestrator_engine", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        print(f"Server Timeout: {e}. Check VLM and Perception/MTC nodes.")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    # Initialize Blackboard variable
    blackboard = py_trees.blackboard.Client(name="Main_Engine")
    blackboard.register_key(key="mission_completed", access=py_trees.common.Access.WRITE)
    blackboard.mission_completed = False

    print("\n--- Dynamic Orchestrator Initialized ---")
    
    try:
        tree.tick_tock(period_ms=1000) # Throttled to 1s for cleaner logging
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()