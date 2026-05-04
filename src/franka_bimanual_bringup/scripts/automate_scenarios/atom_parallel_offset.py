#!/usr/bin/env python3

import os
import sys
import rclpy
import py_trees
import py_trees_ros

# Add path for behaviors
workspace_path = "/home/hargalaten/Documents/vfalcini/franka_prisma_master_thesis/src"
# Map /mm_ws if in docker
if os.path.exists("/mm_ws/src"):
    workspace_path = "/mm_ws/src"

sys.path.append(os.path.join(workspace_path, "franka_bimanual_orchestrator"))

from franka_bimanual_orchestrator.behaviors.move_home_client import MoveHomeClient

def create_parallel_offset_tree():
    root = py_trees.composites.Parallel(
        name="Parallel_Offset_Test",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    
    # Left Arm -> ready
    left_home = MoveHomeClient(name="Left_Arm_Ready", prefix="left_")
    # Right Arm -> offset_ready (small movement)
    right_home = MoveHomeClient(name="Right_Arm_Offset", prefix="right_")
    
    root.add_children([left_home, right_home])
    return root

def main():
    rclpy.init()
    
    # 1. Initialize Blackboard
    blackboard = py_trees.blackboard.Client(name="ParallelTest")
    blackboard.register_key(key="left_active_arm", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_active_arm", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="left_target_pose_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_target_pose_name", access=py_trees.common.Access.WRITE)
    
    blackboard.left_active_arm = "left_arm"
    blackboard.right_active_arm = "right_arm"
    blackboard.left_target_pose_name = "offset_ready"
    blackboard.right_target_pose_name = "offset_ready"

    # 2. Build and run the tree
    root = create_parallel_offset_tree()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    
    tree.visitors.append(py_trees.visitors.DisplaySnapshotVisitor())
    
    print("🚀 Running PARALLEL OFFSET Test (Left: ready, Right: offset_ready)...")
    print("This will demonstrate independent and simultaneous motion of both arms.")
    
    try:
        tree.setup(node_name="parallel_offset_engine", timeout=15.0)
        
        while rclpy.ok():
            tree.tick_tock(period_ms=500, number_of_iterations=1)
            
            if tree.root.status == py_trees.common.Status.SUCCESS:
                print("\n✅ MISSION COMPLETED: Both arms reached their targets in parallel!")
                break
            
            if tree.root.status == py_trees.common.Status.FAILURE:
                print("\n❌ MISSION FAILED: Check logs.")
                break
                
            rclpy.spin_once(tree.node, timeout_sec=0.1)

    except KeyboardInterrupt:
        print("\n🛑 Manual Interruption.")
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
