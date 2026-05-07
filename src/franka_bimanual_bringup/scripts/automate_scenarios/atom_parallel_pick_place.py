#!/usr/bin/env python3

import os
import sys
import rclpy
import py_trees
import py_trees_ros.trees
import time
import operator

# Path Setup
workspace_path = "/home/hargalaten/Documents/vfalcini/franka_prisma_master_thesis/src"
if os.path.exists("/mm_ws/src"):
    workspace_path = "/mm_ws/src"

sys.path.append(os.path.join(workspace_path, "franka_bimanual_orchestrator"))

from franka_bimanual_orchestrator.behaviors.pick_client import PickActionClient
from franka_bimanual_orchestrator.behaviors.place_client import PlaceActionClient
from franka_bimanual_orchestrator.behaviors.move_home_client import MoveHomeClient

def create_parallel_task_tree():
    # --- TOP LEVEL SEQUENCE ---
    main_root = py_trees.composites.Sequence(name="Main_Root", memory=True)

    # ------------------------------------------------------
    # PHASE 1: ASYNC HOME (Both arms move to home simultaneously)
    # ------------------------------------------------------
    initial_home = py_trees.composites.Parallel(
        name="Initial_Async_Home",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    home_left = MoveHomeClient(name="Home_Left", action_name="/move_home", prefix="left_")
    home_right = MoveHomeClient(name="Home_Right", action_name="/move_home", prefix="right_")
    initial_home.add_children([home_left, home_right])

    # ------------------------------------------------------
    # PHASE 2: SCENARIO EXECUTION (The Parallel Relay)
    # ------------------------------------------------------
    scenario_parallel = py_trees.composites.Parallel(
        name="Scenario_Parallel_Execution",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    
    # RIGHT ARM: Pick from 'target_object', Place in 'shared'
    right_seq = py_trees.composites.Sequence(name="Right_Task_Sequence", memory=True)
    # ... (right_seq setup continues below)
    
    # 1. Setup Right Target for Pick
    right_set_pick = py_trees.behaviours.SetBlackboardVariable(
        name="RT_Set_Pick_Tgt", variable_name="right_target_name", variable_value="target_object", overwrite=True)
    right_pick = PickActionClient(name="Right_Pick", action_name="/pick_object", prefix="right_")
    
    # 2. Setup Right Target for Place WITH MUTEX
    right_place_seq = py_trees.composites.Sequence(name="Right_Place_Protected", memory=True)
    
    # Wait for Mutex to be free (Using Decorated Check)
    right_check_free = py_trees.behaviours.CheckBlackboardVariableValue(
        name="RT_Check_Mutex_Free",
        check=py_trees.common.ComparisonExpression(
            variable="shared_workspace_lock", value=False, operator=operator.eq)
    )
    right_wait_mutex = py_trees.decorators.FailureIsRunning(
        name="RT_Wait_Mutex", child=right_check_free)
    
    # Lock Mutex
    right_lock_mutex = py_trees.behaviours.SetBlackboardVariable(
        name="RT_Lock_Workspace", variable_name="shared_workspace_lock", variable_value=True, overwrite=True)
    
    right_set_place = py_trees.behaviours.SetBlackboardVariable(
        name="RT_Set_Place_Tgt", variable_name="right_target_name", variable_value="shared", overwrite=True)
    
    right_place = PlaceActionClient(name="Right_Place", action_name="/place_object", prefix="right_")
    
    # Unlock Mutex
    right_unlock_mutex = py_trees.behaviours.SetBlackboardVariable(
        name="RT_Unlock_Workspace", variable_name="shared_workspace_lock", variable_value=False, overwrite=True)
    
    # Signal that object is ready for pick
    right_signal_ready = py_trees.behaviours.SetBlackboardVariable(
        name="RT_Signal_Object_Ready", variable_name="object_ready_at_shared", variable_value=True, overwrite=True)
    
    right_place_seq.add_children([right_wait_mutex, right_lock_mutex, right_set_place, right_place, right_unlock_mutex, right_signal_ready])
    
    # 3. Go Home
    right_home = MoveHomeClient(name="Right_Home", action_name="/move_home", prefix="right_")
    
    right_seq.add_children([right_set_pick, right_pick, right_place_seq, right_home])

    # ------------------------------------------------------
    # LEFT ARM: Pick from 'shared', Place in 'box' (RELAY)
    # ------------------------------------------------------
    left_seq = py_trees.composites.Sequence(name="Left_Task_Sequence", memory=True)
    
    # 1. Setup Left Target for Pick FROM SHARED (Protected by Mutex)
    left_pick_seq = py_trees.composites.Sequence(name="Left_Pick_Protected", memory=True)
    
    # Wait for Object to be placed at shared (Using Decorated Check)
    left_check_obj = py_trees.behaviours.CheckBlackboardVariableValue(
        name="LT_Check_Obj_Ready",
        check=py_trees.common.ComparisonExpression(
            variable="object_ready_at_shared", value=True, operator=operator.eq)
    )
    left_wait_object = py_trees.decorators.FailureIsRunning(
        name="LT_Wait_Object", child=left_check_obj)

    # Wait for Mutex to be free (Using Decorated Check)
    left_check_free = py_trees.behaviours.CheckBlackboardVariableValue(
        name="LT_Check_Mutex_Free",
        check=py_trees.common.ComparisonExpression(
            variable="shared_workspace_lock", value=False, operator=operator.eq)
    )
    left_wait_mutex = py_trees.decorators.FailureIsRunning(
        name="LT_Wait_Mutex", child=left_check_free)
    
    # Lock Mutex to pick safely
    left_lock_mutex = py_trees.behaviours.SetBlackboardVariable(
        name="LT_Lock_Workspace", variable_name="shared_workspace_lock", variable_value=True, overwrite=True)
    
    left_set_pick = py_trees.behaviours.SetBlackboardVariable(
        name="LT_Set_Pick_Tgt", variable_name="left_target_name", variable_value="shared", overwrite=True)
    
    left_pick = PickActionClient(name="Left_Pick", action_name="/pick_object", prefix="left_")
    
    # Unlock after picking
    left_unlock_mutex = py_trees.behaviours.SetBlackboardVariable(
        name="LT_Unlock_Workspace", variable_name="shared_workspace_lock", variable_value=False, overwrite=True)
    
    left_pick_seq.add_children([left_wait_object, left_wait_mutex, left_lock_mutex, left_set_pick, left_pick, left_unlock_mutex])
    
    # 2. Setup Left Target for Place in BOX
    left_set_place = py_trees.behaviours.SetBlackboardVariable(
        name="LT_Set_Place_Tgt", variable_name="left_target_name", variable_value="box", overwrite=True)
    
    left_place = PlaceActionClient(name="Left_Place", action_name="/place_object", prefix="left_")
    
    # 3. Go Home
    left_home = MoveHomeClient(name="Left_Home", action_name="/move_home", prefix="left_")
    
    left_seq.add_children([left_pick_seq, left_set_place, left_place, left_home])

    scenario_parallel.add_children([right_seq, left_seq])
    main_root.add_children([initial_home, scenario_parallel])
    return main_root

def main():
    rclpy.init()
    node = rclpy.create_node("parallel_task_orchestrator")
    
    # Initialize Blackboard
    blackboard = py_trees.blackboard.Client()
    blackboard.register_key(key="left_active_arm", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_active_arm", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="left_target_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_target_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="left_target_pose_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_target_pose_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="shared_workspace_lock", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="object_ready_at_shared", access=py_trees.common.Access.WRITE)

    blackboard.left_active_arm = "left_arm"
    blackboard.right_active_arm = "right_arm"
    blackboard.left_target_pose_name = "ready"
    blackboard.right_target_pose_name = "ready"
    blackboard.shared_workspace_lock = False
    blackboard.object_ready_at_shared = False

    # Create and setup tree
    root = create_parallel_task_tree()
    tree = py_trees_ros.trees.BehaviourTree(root)
    
    # Add snapshot visitor for automatic tree printing (like in offset script)
    tree.visitors.append(py_trees.visitors.DisplaySnapshotVisitor())
    
    tree.setup(timeout=15.0, node=node)
    
    print("\n🚀 Starting PARALLEL SHARED RELAY Scenario...")
    print("-------------------------------------------------------")
    print("Right Arm: target_object -> shared")
    print("Left Arm: shared -> box (waiting for Right)")
    
    try:
        tick_count = 0
        while rclpy.ok():
            tree.tick()
            tick_count += 1
            # The Visitor handles printing now
                
            if root.status == py_trees.common.Status.SUCCESS:
                print("\n✅ PARALLEL TASK COMPLETED SUCCESSFULLY!")
                break
            if root.status == py_trees.common.Status.FAILURE:
                print("\n❌ PARALLEL TASK FAILED.")
                break
            rclpy.spin_once(node, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
