#!/usr/bin/env python3

import os
import sys
import rclpy
import py_trees
import py_trees_ros

# Add path for behaviors
workspace_path = "/home/falco_robotics/vf_projects_portfolio/mm_ws/src"
# Map /mm_ws if in docker
if os.path.exists("/mm_ws/src"):
    workspace_path = "/mm_ws/src"

sys.path.append(os.path.join(workspace_path, "franka_bimanual_orchestrator"))

from franka_bimanual_orchestrator.behaviors.move_home_client import MoveHomeClient

def create_atomic_tree():
    root = py_trees.composites.Parallel(
        name="Atomic_Sync_Home",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    
    left_home = MoveHomeClient(name="Left_Arm_Home", prefix="left_")
    right_home = MoveHomeClient(name="Right_Arm_Home", prefix="right_")
    
    root.add_children([left_home, right_home])
    return root

def main():
    rclpy.init()
    
    # 1. Initialize Blackboard with hardcoded values to bypass complex logic
    blackboard = py_trees.blackboard.Client(name="Atomic")
    blackboard.register_key(key="left_active_arm", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_active_arm", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="left_target_pose_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_target_pose_name", access=py_trees.common.Access.WRITE)
    
    blackboard.left_active_arm = "left_arm"
    blackboard.right_active_arm = "right_arm"
    blackboard.left_target_pose_name = "ready"
    blackboard.right_target_pose_name = "ready"

    # 2. Build and run the MINIMAL tree
    root = create_atomic_tree()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    
    # Enable viewer support
    tree.visitors.append(py_trees.visitors.DisplaySnapshotVisitor())
    
    print("🚀 Running ATOMIC Synchronization Test (Minimal Tree)...")
    
    try:
        tree.setup(node_name="atomic_home_engine", timeout=15.0)
        
        # Manual loop to allow clean termination on Success
        while rclpy.ok():
            tree.tick_tock(period_ms=500, number_of_iterations=1)
            
            if tree.root.status == py_trees.common.Status.SUCCESS:
                print("\n✅ MISSION COMPLETED: Both arms reached Home!")
                break
            
            if tree.root.status == py_trees.common.Status.FAILURE:
                print("\n❌ MISSION FAILED: Check logs.")
                break
                
            # Allow ROS to process callbacks
            rclpy.spin_once(tree.node, timeout_sec=0.1)

    except KeyboardInterrupt:
        print("\n🛑 Manual Interruption.")
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
