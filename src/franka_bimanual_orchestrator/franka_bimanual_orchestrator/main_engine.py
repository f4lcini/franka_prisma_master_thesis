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
from franka_bimanual_orchestrator.behaviors.rendezvous_client import RendezvousClient
from franka_bimanual_orchestrator.behaviors.planner_utils import PlanSplitter, DynamicActionIterator, PlanPopper

def create_dynamic_arm_sequence(arm_name, plan_steps):
    """
    Builds a pure BT sequence from a list of plan steps.
    """
    prefix = f"{arm_name}_"
    seq = py_trees.composites.Sequence(name=f"Sequence_{arm_name.upper()}", memory=True)
    
    if not plan_steps:
        return py_trees.behaviours.Success(name=f"No_Task_{arm_name}")

    for i, step in enumerate(plan_steps):
        action = step.get('action')
        target = step.get('target_name') or step.get('target_location')
        
        # Mapping string actions to BT Nodes with direct parameter passing
        if action == "FIND_OBJECT":
            # Determinazione del server corretto in base al braccio
            action_server = "/detect_object_left" if arm_name == "left" else "/detect_object_right"
            node = ObjectLocalizationClient(
                name=f"Find_{target}_{i}", 
                prefix=prefix, 
                target_name=target,
                action_name=action_server
            )
        elif action == "PICK":
            node = PickActionClient(name=f"Pick_{target}_{i}", prefix=prefix, target_name=target)
        elif action == "PLACE":
            node = PlaceActionClient(name=f"Place_{target}_{i}", prefix=prefix, target_location=target)
        elif action == "MOVE_HOME":
            node = MoveHomeClient(name=f"Home_{i}", prefix=prefix)
        elif action == "WAIT":
            node = WaitActionClient(name=f"Wait_{i}", prefix=prefix)
        elif action == "RENDEZVOUS":
            node = RendezvousClient(name=f"Rendezvous_{i}", role="donor" if arm_name == "right" else "recipient")
        else:
            continue

        # Inseriamo i parametri del target nella blackboard per quel nodo specifico
        # Nota: In un albero dinamico, possiamo anche passare i parametri direttamente al costruttore 
        # se modifichiamo i nodi, ma per ora usiamo la logica esistente.
        seq.add_child(node)
        
    return seq

def create_tree(task_description, full_plan):
    """
    Constructs the tree dynamically based on the plan.
    """
    root = py_trees.composites.Sequence(name=f"Mission: {task_description}", memory=True)
    
    # 1. Parallel execution of arm sequences
    execution_parallel = py_trees.composites.Parallel(
        name="Bimanual_Execution",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    
    left_seq = create_dynamic_arm_sequence("left", full_plan.get('left_arm_sequence', []))
    right_seq = create_dynamic_arm_sequence("right", full_plan.get('right_arm_sequence', []))
    
    execution_parallel.add_children([left_seq, right_seq])
    
    root.add_child(execution_parallel)
    return root

import json

def main():
    rclpy.init(args=sys.argv)
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("task", type=str, nargs="?", default="Bimanual Operation")
    parser.add_argument("--plan", type=str, help="Path to a JSON file containing a custom plan")
    parser.add_argument("--object", type=str, help="Override the target object name in the plan")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])

    # 1. Load Plan
    full_plan = {}
    task_desc = args.task
    if args.plan:
        try:
            with open(args.plan, 'r') as f:
                full_plan = json.load(f)
            print(f"✅ Custom plan loaded from {args.plan}.")
            
            # --- BLACKBOARD INITIALIZATION ---
            # Sempre inizializzata per evitare KeyError nei comportamenti
            blackboard = py_trees.blackboard.Client(name="MainConfig")
            blackboard.register_key(key="left_target_name", access=py_trees.common.Access.WRITE)
            blackboard.register_key(key="right_target_name", access=py_trees.common.Access.WRITE)
            blackboard.register_key(key="left_active_arm", access=py_trees.common.Access.WRITE)
            blackboard.register_key(key="right_active_arm", access=py_trees.common.Access.WRITE)
            blackboard.register_key(key="left_target_pose_name", access=py_trees.common.Access.WRITE)
            blackboard.register_key(key="right_target_pose_name", access=py_trees.common.Access.WRITE)
            blackboard.register_key(key="left_target_location", access=py_trees.common.Access.WRITE)
            blackboard.register_key(key="right_target_location", access=py_trees.common.Access.WRITE)
            blackboard.register_key(key="mission_metadata", access=py_trees.common.Access.WRITE)
            blackboard.register_key(key="mission_type", access=py_trees.common.Access.WRITE)
            
            # Valori di default
            blackboard.left_target_name = "none"
            blackboard.right_target_name = "none"
            blackboard.left_active_arm = "left_arm"
            blackboard.right_active_arm = "right_arm"
            blackboard.left_target_pose_name = "ready"
            blackboard.right_target_pose_name = "ready"
            blackboard.left_target_location = "box"
            blackboard.right_target_location = "box"
            blackboard.mission_metadata = {}
            blackboard.mission_type = "SIMPLE"
            
            # --- OVERRIDE LOGIC ---
            if args.object:
                print(f"🔄 Overriding all targets with: {args.object}")
                blackboard.left_target_name = args.object
                blackboard.right_target_name = args.object
                
                for arm in ['left_arm_sequence', 'right_arm_sequence']:
                    if arm in full_plan:
                        for step in full_plan[arm]:
                            # Caso 1: target_name alla radice dello step
                            if 'target_name' in step and step['target_name'] not in ['shared', 'box']:
                                step['target_name'] = args.object
                            # Caso 2: target_name dentro config
                            if 'config' in step and 'target_name' in step['config'] and step['config']['target_name'] not in ['shared', 'box']:
                                step['config']['target_name'] = args.object
            # ----------------------
            
        except Exception as e:
            print(f"❌ Failed to load plan: {e}")
            return
    else:
        print("❌ No plan provided. Please use --plan <file.json>")
        return

    # 2. Build Tree
    root = create_tree(task_desc, full_plan)
    # Add OneShot decorator to prevent infinite looping
    root = py_trees.decorators.OneShot(
        name="Single Mission", 
        child=root, 
        policy=py_trees.common.OneShotPolicy.ON_COMPLETION
    )

    print("\n" + "="*40)
    print("🎄 GENERATED BEHAVIOR TREE:")
    print("="*40)
    print(py_trees.display.ascii_tree(root))
    print("="*40 + "\n")
    
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    
    # 3. Setup (Basic blackboard for shared flags)
    blackboard = py_trees.blackboard.Client(name="Main")
    blackboard.register_key(key="handover_ready", access=py_trees.common.Access.WRITE)
    blackboard.handover_ready = False

    try:
        tree.setup(node_name="bimanual_engine", timeout=15.0)
    except Exception as e:
        print(f"Setup failed: {e}")
        return

    print("\n--- Bimanual DYNAMIC Engine Ready ---")
    
    try:
        # Tick at 1Hz
        tree.tick_tock(period_ms=1000)
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()