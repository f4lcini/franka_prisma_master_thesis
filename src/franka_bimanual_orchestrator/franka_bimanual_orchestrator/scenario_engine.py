#!/usr/bin/env python3

import rclpy
import py_trees
import py_trees_ros
import sys
import operator
import json
from rclpy.node import Node

from franka_bimanual_orchestrator.behaviors.object_localization_client import ObjectLocalizationClient
from franka_bimanual_orchestrator.behaviors.pick_client import PickActionClient
from franka_bimanual_orchestrator.behaviors.place_client import PlaceActionClient
from franka_bimanual_orchestrator.behaviors.move_home_client import MoveHomeClient
from franka_bimanual_orchestrator.behaviors.wait_client import WaitActionClient
from franka_bimanual_orchestrator.behaviors.give_client import GiveActionClient
from franka_bimanual_orchestrator.behaviors.take_client import TakeActionClient
from franka_bimanual_orchestrator.behaviors.planner_utils import PlanSplitter, DynamicActionIterator, PlanPopper

def create_arm_lane(arm_name="left"):
    prefix = f"{arm_name}_"
    plan_key = f"{arm_name}_arm_plan"
    
    step = py_trees.composites.Sequence(name=f"Step_{arm_name}", memory=True)
    
    has_plan = py_trees.behaviours.CheckBlackboardVariableValue(
        name=f"Has_{arm_name}_Plan?",
        check=py_trees.common.ComparisonExpression(
            variable=plan_key,
            value=[],
            operator=operator.ne
        )
    )
    
    iterator = DynamicActionIterator(name=f"Iterator_{arm_name}", plan_key=plan_key, prefix=prefix)
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
        
    popper = PlanPopper(name=f"Popper_{arm_name}", plan_key=plan_key)
    step.add_children([has_plan, iterator, dispatcher, popper])
    
    repeat_loop = py_trees.decorators.Repeat(
        child=step,
        num_success=-1,
        name=f"Repeat_{arm_name.upper()}"
    )
    
    lane = py_trees.decorators.FailureIsSuccess(
        child=repeat_loop,
        name=f"Lane_{arm_name.upper()}"
    )
    
    return lane

def create_scenario_tree(plan):
    root = py_trees.composites.Selector(name="Scenario_Root", memory=True)
    
    mission_done = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Mission_Done?",
        check=py_trees.common.ComparisonExpression(variable="mission_completed", value=True, operator=operator.eq)
    )

    main_sequence = py_trees.composites.Sequence(name="Execution_Sequence", memory=True)
    
    # 1. SETUP PLAN
    setup_plan = py_trees.behaviours.SetBlackboardVariable(
        name="Load_Scenario_Plan",
        variable_name="vlm_plan",
        variable_value=plan,
        overwrite=True
    )

    splitter = PlanSplitter(name="Plan_Splitter")
    
    # 2. PARALLEL EXECUTION
    execution_parallel = py_trees.composites.Parallel(
        name="Parallel_Arms_Execution",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    execution_parallel.add_children([create_arm_lane("left"), create_arm_lane("right")])
    
    # 3. FINALIZE
    mark_done = py_trees.behaviours.SetBlackboardVariable(
        name="Mark_Mission_Complete",
        variable_name="mission_completed",
        variable_value=True,
        overwrite=True
    )

    main_sequence.add_children([setup_plan, splitter, execution_parallel, mark_done])
    root.add_children([mission_done, main_sequence])
    
    return root

def run_scenario(plan, node_name="scenario_engine"):
    rclpy.init()
    
    root = create_scenario_tree(plan)
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    
    # Adding Snapshot Visitor enables visualizers (CLI and GUI) to see the tree state
    tree.visitors.append(py_trees.visitors.DisplaySnapshotVisitor())
    
    try:
        tree.setup(node_name=node_name, timeout=15.0)
    except Exception as e:
        print(f"Setup failed: {e}")
        return

    blackboard = py_trees.blackboard.Client(name="Scenario")
    blackboard.register_key(key="mission_completed", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="vlm_plan", access=py_trees.common.Access.WRITE)
    
    blackboard.mission_completed = False
    blackboard.vlm_plan = None

    print(f"\n🚀 Scenario Engine [{node_name}] Starting...")
    print(f"Plan Actions: {len(plan)}")
    
    try:
        tree.tick_tock(period_ms=500)
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

def main():
    # Default test plan if run directly
    test_plan = [
        {"action": "WAIT", "arm": "left_arm", "message": "Test Left"},
        {"action": "WAIT", "arm": "right_arm", "message": "Test Right"}
    ]
    run_scenario(test_plan)

if __name__ == '__main__':
    main()
