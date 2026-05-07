#!/usr/bin/env python3

import os
import sys
import rclpy
import py_trees
import py_trees_ros
import operator
import time

# Path Setup
workspace_path = "/home/hargalaten/Documents/vfalcini/franka_prisma_master_thesis/src"
if os.path.exists("/mm_ws/src"):
    workspace_path = "/mm_ws/src"

sys.path.append(os.path.join(workspace_path, "franka_bimanual_orchestrator"))

from franka_bimanual_orchestrator.behaviors.pick_client import PickActionClient
from franka_bimanual_orchestrator.behaviors.place_client import PlaceActionClient
from franka_bimanual_orchestrator.behaviors.move_home_client import MoveHomeClient

def create_intermediate_test_tree():
    # Only the parallel task, no nested sequences for now
    task_parallel = py_trees.composites.Parallel(
        name="Parallel_Pick_Place",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    
    # Right: Pick
    right_pick = PickActionClient(name="Right_Pick", action_name="/pick_object", prefix="right_")
    
    # Left: Place
    left_place = PlaceActionClient(name="Left_Place", action_name="/place_object", prefix="left_")
    
    task_parallel.add_children([right_pick, left_place])
    return task_parallel

def main():
    rclpy.init()
    node = rclpy.create_node("intermediate_test_orchestrator")
    
    blackboard = py_trees.blackboard.Client(name="IntermediateTest")
    # Register keys
    for key in ["left_active_arm", "right_active_arm", "left_target_location", "right_target_name", "left_target_pose_name", "right_target_pose_name", "handover_ready", "handover_starting"]:
        blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
    
    blackboard.left_active_arm = "left_arm"
    blackboard.right_active_arm = "right_arm"
    blackboard.right_target_name = "target_object"
    blackboard.left_target_location = "box"
    blackboard.left_target_pose_name = "ready"
    blackboard.right_target_pose_name = "ready"
    blackboard.handover_ready = False
    blackboard.handover_starting = False

    # Build and run the tree
    root = create_intermediate_test_tree()
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.visitors.append(py_trees.visitors.DisplaySnapshotVisitor())
    
    print("🚀 Running INTERMEDIATE PARALLEL Test...")
    print("1. Right: Pick 'target_object' | Left: Place 'box' (Parallel)")
    print("2. Both: Move Home (Parallel)")
    
    try:
        tree.setup(timeout=15.0, node=node)
        
        print("\n--- Starting Tree Execution ---")
        done = False
        while rclpy.ok() and not done:
            tree.tick()
            
            status = root.status
            if status == py_trees.common.Status.SUCCESS:
                print("\n✅ MISSION ACCOMPLISHED: All tasks finished successfully!")
                done = True
            elif status == py_trees.common.Status.FAILURE:
                print("\n❌ MISSION FAILED: Tree returned failure status.")
                done = True
            
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.1)

        print("\nFinal Tree State:")
        print(py_trees.display.unicode_tree(root))

    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
