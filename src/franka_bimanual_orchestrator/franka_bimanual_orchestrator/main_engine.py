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
import time
import operator

class UnmatchedActionLogger(py_trees.behaviour.Behaviour):
    """
    Diagnostic node that logs when the dispatcher fails to find a matching skill.
    """
    def __init__(self, name="Diagnostic_Fallback", prefix="left_"):
        super().__init__(name=name)
        self.prefix = prefix
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=f"{prefix}active_action", access=py_trees.common.Access.READ)

    def update(self):
        action = getattr(self.blackboard, f"{self.prefix}active_action", "NONE")
        self.logger.error(f"❌ [DISPATCHER ERROR] No skills matched current action: '{action}' for {self.prefix[:-1]}!")
        return py_trees.common.Status.FAILURE

from franka_bimanual_orchestrator.behaviors.vlm_client import VlmActionClient
from franka_bimanual_orchestrator.behaviors.object_localization_client import ObjectLocalizationClient
from franka_bimanual_orchestrator.behaviors.pick_client import PickActionClient
from franka_bimanual_orchestrator.behaviors.place_client import PlaceActionClient
from franka_bimanual_orchestrator.behaviors.move_home_client import MoveHomeClient
from franka_bimanual_orchestrator.behaviors.wait_client import WaitActionClient
from franka_bimanual_orchestrator.behaviors.give_client import GiveActionClient
from franka_bimanual_orchestrator.behaviors.take_client import TakeActionClient
from franka_bimanual_orchestrator.behaviors.planner_utils import PlanSplitter, DynamicActionIterator, PlanPopper
from franka_bimanual_orchestrator.behaviors.recovery_utils import MissionAbort, ReplanTrigger, MissionFail, MissionExit

def create_arm_lane(arm_name="left"):
    """
    Creates a namespaced sequence lane for a specific arm.
    Repeats until the arm-specific plan is empty.
    """
    prefix = f"{arm_name}_"
    plan_key = f"{arm_name}_arm_plan"
    
    # 1. Action Step (Sequence) - No memory to ensure loops refresh correctly
    step = py_trees.composites.Sequence(name=f"Step_{arm_name}", memory=False)
    
    # 1.1 Condition: Plan not empty (Returns FAILURE when plan is empty)
    def create_plan_check(name_suffix):
        return py_trees.behaviours.CheckBlackboardVariableValue(
            name=f"Has_{arm_name}_Plan_{name_suffix}?",
            check=py_trees.common.ComparisonExpression(
                variable=plan_key,
                value=[],
                operator=operator.ne
            )
        )
    
    # 1.2 Iterator
    iterator = DynamicActionIterator(name=f"Iterator_{arm_name}", plan_key=plan_key, prefix=prefix)
    
    # 1.3 Action Dispatcher (Non-memory Selector)
    dispatcher = py_trees.composites.Selector(name=f"Dispatcher_{arm_name}", memory=False)
    
    # Skills Library mapping
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
        skill_seq = py_trees.composites.Sequence(name=f"{skill_name}_{arm_name}_Seq", memory=False)
        is_active = py_trees.behaviours.CheckBlackboardVariableValue(
            name=f"Is_{skill_name}_{arm_name}?",
            check=py_trees.common.ComparisonExpression(
                variable=f"{prefix}active_action",
                value=skill_name,
                operator=operator.eq
            )
        )
        
        # Skill with safety valve
        skill_execution = py_trees.composites.Selector(name=f"{skill_name}_Execution_{arm_name}", memory=False)
        skill_execution.add_child(client_node)
        
        # Recovery Selector: Replan vs Fatal Abort
        recovery_selector = py_trees.composites.Selector(name=f"{skill_name}_Recovery_{arm_name}", memory=False)
        
        # Branch A: REPLAN if object missed (PICK or TAKE)
        if skill_name in ["PICK", "TAKE", "FIND_OBJECT"]:
            replan_branch = py_trees.composites.Sequence(name=f"Replan_Branch_{arm_name}", memory=False)
            
            # --- Robust Error Check Selector (Avoids TypeError with operator.contains) ---
            error_selector = py_trees.composites.Selector(name=f"Check_Error_{arm_name}", memory=False)
            
            error_keywords = ["Object missed during grasp", "Object missed during take", "missing_pos", "No object detected"]
            for keyword in error_keywords:
                check_node = py_trees.behaviours.CheckBlackboardVariableValue(
                    name=f"Error_is_{keyword.replace(' ', '_')}_{arm_name}?",
                    check=py_trees.common.ComparisonExpression(
                        variable=f"{prefix}last_error",
                        value=keyword,
                        operator=operator.eq
                    )
                )
                error_selector.add_child(check_node)
            
            replan_trigger = ReplanTrigger(name=f"Trigger_Replan_{arm_name}")
            error_home_soft = MoveHomeClient(name=f"Soft_Home_{arm_name}", prefix=prefix)
            replan_branch.add_children([error_selector, error_home_soft, replan_trigger])
            recovery_selector.add_child(replan_branch)

        # Branch B: FATAL ABORT (IK Failure, Move Rejection, etc.)
        fatal_abort = py_trees.composites.Sequence(name=f"Fatal_Abort_{arm_name}", memory=False)
        error_home_fatal = MoveHomeClient(name=f"Safety_Home_{arm_name}", prefix=prefix)
        abort_signal = MissionAbort(name=f"Abort_from_{skill_name}_{arm_name}")
        fatal_fail = MissionFail(name=f"Terminal_Fail_{arm_name}")
        fatal_abort.add_children([error_home_fatal, abort_signal, fatal_fail])
        
        recovery_selector.add_child(fatal_abort)
        skill_execution.add_child(recovery_selector)
        
        skill_seq.add_children([is_active, skill_execution])
        dispatcher.add_child(skill_seq)
        
    # --- DIAGNOSTIC FALLBACK ---
    dispatcher.add_child(UnmatchedActionLogger(name=f"Fallback_{arm_name}", prefix=prefix))

    # 1.4 Popper
    popper = PlanPopper(name=f"Popper_{arm_name}", plan_key=plan_key)
    
    step.add_children([create_plan_check("Loop"), iterator, dispatcher, popper])
    
    # 2. Operational Sequence: Repeat steps until has_plan fails (plan empty)
    op_seq = py_trees.composites.Sequence(name=f"Op_Seq_{arm_name}", memory=False)
    
    # Wrap Repeat to ensure it finishes normally
    repeat_loop = py_trees.decorators.Repeat(
        child=step,
        num_success=-1,
        name=f"Loop_{arm_name.upper()}"
    )
    
    op_seq.add_children([create_plan_check("Gate"), repeat_loop])
    
    # 3. Final Lane Structure (Selector)
    lane = py_trees.composites.Selector(name=f"Lane_{arm_name}", memory=False)
    
    # Recovery and Retry logic would go inside the dispatcher or here.
    # For now, we ensure the arm moves HOME after the operational sequence finishes (fails due to empty plan).
    final_home = MoveHomeClient(name=f"Final_Home_{arm_name}", prefix=prefix)
    
    lane.add_children([op_seq, final_home])
    
    return lane

def create_tree(task_description="Default Task"):
    # --- ROOT SELECTOR ---
    root = py_trees.composites.Selector(name="Root_Guardian", memory=False)
    
    # 1. MISSION DONE GUARD
    mission_done = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Mission_Completed?",
        check=py_trees.common.ComparisonExpression(variable="mission_completed", value=True, operator=operator.eq)
    )

    # 2. EMERGENCY STOP GUARD (Terminal state)
    emergency_branch = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Mission_Aborted?",
        check=py_trees.common.ComparisonExpression(variable="mission_aborted", value=True, operator=operator.eq)
    )

    # 3. MISSION REPEAT LOOP
    # This loop keeps ticking until mission_completed or mission_aborted is set.
    main_loop = py_trees.composites.Sequence(name="Mission_Loop", memory=True)

    # 3.1 Workflow Sequence
    workflow = py_trees.composites.Sequence(name="Bimanual_Orchestrator", memory=True)
    
    # - Planning Gate
    planning_gate = py_trees.composites.Selector(name="Planning_Gate", memory=True)
    has_plan_already = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Plan_Exists?",
        check=py_trees.common.ComparisonExpression(variable="vlm_plan", value=None, operator=operator.ne)
    )
    vlm_planner = VlmActionClient(name="Gemini_Planner", task_description=task_description)
    planning_gate.add_children([has_plan_already, vlm_planner])

    splitter = PlanSplitter(name="Plan_Splitter")
    
    # - Parallel Lanes
    execution_parallel = py_trees.composites.Parallel(
        name="Side_By_Side_Execution",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    execution_parallel.add_children([create_arm_lane("left"), create_arm_lane("right")])
    
    # - Success Latching
    mark_done = py_trees.behaviours.SetBlackboardVariable(
        name="Set_Complete",
        variable_name="mission_completed",
        variable_value=True,
        overwrite=True
    )
    mission_exit = MissionExit(name="Final_Shutdown")

    workflow.add_children([planning_gate, splitter, execution_parallel, mark_done, mission_exit])
    
    # No decorator needed here; Root_Guardian will tick 'workflow' again on next loop if it fails/restarts.
    main_loop.add_child(workflow)

    root.add_children([mission_done, emergency_branch, main_loop])
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
    blackboard.register_key(key="mission_aborted", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="abort_message", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="vlm_plan", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="handover_ready", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="handover_starting", access=py_trees.common.Access.WRITE)
    
    blackboard.mission_completed = False
    blackboard.mission_aborted = False
    blackboard.abort_message = ""
    blackboard.vlm_plan = None
    blackboard.handover_ready = False
    blackboard.handover_starting = False

    print("\n--- Bimanual Parallel Engine Ready (Robust Looping Version) ---")
    
    try:
        # --- INCREASED FREQUENCY (5Hz) for Interrogazione Spinta ---
        tree.tick_tock(period_ms=200)
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()