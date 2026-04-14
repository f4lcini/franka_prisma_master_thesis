#!/usr/bin/env python3

"""
================================================================================
Author: Falco Robotics 
Code Description: 
Dual-Arm Franka Orchestrator with Logic-based Synchronization and Fixed Looping.
================================================================================
"""

import rclpy
import py_trees
import py_trees_ros
import sys
import operator

from franka_bimanual_orchestrator.behaviors.vlm_client import VlmActionClient
from franka_bimanual_orchestrator.behaviors.object_localization_client import ObjectLocalizationClient
from franka_bimanual_orchestrator.behaviors.pick_client import PickActionClient
from franka_bimanual_orchestrator.behaviors.place_client import PlaceActionClient
from franka_bimanual_orchestrator.behaviors.move_home_client import MoveHomeClient
from franka_bimanual_orchestrator.behaviors.wait_client import WaitActionClient
from franka_bimanual_orchestrator.behaviors.give_client import GiveActionClient
from franka_bimanual_orchestrator.behaviors.take_client import TakeActionClient
from franka_bimanual_orchestrator.behaviors.planner_utils import PlanSplitter, DynamicActionIterator, PlanPopper

def create_arm_lane(arm_name="left"):
    """
    Creates a namespaced sequence lane for a specific arm.
    Repeats until the arm-specific plan is empty.
    """
    prefix = f"{arm_name}_"
    plan_key = f"{arm_name}_arm_plan"
    
    # 1. Action Step (Sequence) - MUST return FAILURE when no plan left to exit the repeat
    step = py_trees.composites.Sequence(name=f"Step_{arm_name}", memory=True)
    
    # 1.1 Condition: Plan not empty (Returns FAILURE when plan is empty)
    has_plan = py_trees.behaviours.CheckBlackboardVariableValue(
        name=f"Has_{arm_name}_Plan?",
        check=py_trees.common.ComparisonExpression(
            variable=plan_key,
            value=[],
            operator=operator.ne
        )
    )
    
    # 1.2 Iterator
    iterator = DynamicActionIterator(name=f"Iterator_{arm_name}", plan_key=plan_key, prefix=prefix)
    
    # 1.3 Dispatcher
    dispatcher = py_trees.composites.Selector(name=f"Dispatcher_{arm_name}", memory=False)
    
    available_skills = {
        "FIND_OBJECT": ObjectLocalizationClient(name=f"YOLO_{arm_name}", prefix=prefix),
        "PICK": PickActionClient(name=f"Pick_{arm_name}", prefix=prefix),
        "PLACE": PlaceActionClient(name=f"Place_{arm_name}", prefix=prefix),
        "MOVE_HOME": MoveHomeClient(name=f"Home_{arm_name}", prefix=prefix),
        "WAIT": WaitActionClient(name=f"Wait_{arm_name}", prefix=prefix),
        "GIVE": GiveActionClient(name=f"Give_{arm_name}", prefix=prefix),
        "TAKE": TakeActionClient(name=f"Take_{arm_name}", prefix=prefix)
    }

    for skill_name, client_node in available_skills.items():
        skill_seq = py_trees.composites.Sequence(name=f"{skill_name}_{arm_name}_Seq", memory=True)
        is_active = py_trees.behaviours.CheckBlackboardVariableValue(
            name=f"Is_{skill_name}_{arm_name}?",
            check=py_trees.common.ComparisonExpression(
                variable=f"{prefix}active_action",
                value=skill_name,
                operator=operator.eq
            )
        )
        skill_seq.add_children([is_active, client_node])
        dispatcher.add_child(skill_seq)
        
    # 1.4 Popper
    popper = PlanPopper(name=f"Popper_{arm_name}", plan_key=plan_key)
    
    step.add_children([has_plan, iterator, dispatcher, popper])
    
    # Repeat indefinitely until 'step' fails (i.e. has_plan returns FAILURE)
    repeat_loop = py_trees.decorators.Repeat(
        child=step,
        num_success=-1,
        name=f"Repeat_{arm_name.upper()}"
    )
    
    # When repeat_loop terminates (Failure), we want the lane to return SUCCESS to the Parallel node.
    lane = py_trees.decorators.FailureIsSuccess(
        child=repeat_loop,
        name=f"Lane_{arm_name.upper()}"
    )
    
    return lane

def create_tree(task_description="Default Task"):
    root = py_trees.composites.Selector(name="Root_Guard", memory=True)
    
    # Condition: Stop if mission already marked simple success
    mission_done = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Mission_Done?",
        check=py_trees.common.ComparisonExpression(variable="mission_completed", value=True, operator=operator.eq)
    )

    main_sequence = py_trees.composites.Sequence(name="Bimanual_Orchestrator", memory=True)
    
    # 1. PLANNING GATE
    planning_gate = py_trees.composites.Selector(name="Planning_Gate", memory=False)
    has_plan_already = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Plan_Exists?",
        check=py_trees.common.ComparisonExpression(variable="vlm_plan", value=None, operator=operator.ne)
    )
    vlm_planner = VlmActionClient(name="Gemini_Planner", task_description=task_description)
    planning_gate.add_children([has_plan_already, vlm_planner])

    splitter = PlanSplitter(name="Plan_Splitter")
    
    # 2. PARALLEL EXECUTION
    execution_parallel = py_trees.composites.Parallel(
        name="Side_By_Side_Execution",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    execution_parallel.add_children([create_arm_lane("left"), create_arm_lane("right")])
    
    # 3. FINALIZE (Latching)
    mark_done = py_trees.behaviours.SetBlackboardVariable(
        name="Set_Complete",
        variable_name="mission_completed",
        variable_value=True,
        overwrite=True
    )

    main_sequence.add_children([planning_gate, splitter, execution_parallel, mark_done])
    root.add_children([mission_done, main_sequence])
    
    return root

def main():
    rclpy.init(args=sys.argv)
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("task", type=str, nargs="?", default="Put the red cube in the box")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])

    root = create_tree(task_description=args.task)
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.visitors.append(py_trees.visitors.DisplaySnapshotVisitor())
    
    try:
        tree.setup(node_name="bimanual_engine", timeout=15.0)
    except Exception as e:
        print(f"Setup failed: {e}")
        return

    # INITIALIZE BLACKBOARD
    blackboard = py_trees.blackboard.Client(name="Main")
    blackboard.register_key(key="mission_completed", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="vlm_plan", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="handover_ready", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="handover_starting", access=py_trees.common.Access.WRITE)
    
    blackboard.mission_completed = False
    blackboard.vlm_plan = None
    blackboard.handover_ready = False
    blackboard.handover_starting = False

    print("\n--- Bimanual Parallel Engine Ready (No-Loop Version) ---")
    
    try:
        tree.tick_tock(period_ms=1000)
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()