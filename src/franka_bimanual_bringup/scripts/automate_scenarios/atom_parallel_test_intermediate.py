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

# Import main_engine logic
from franka_bimanual_orchestrator.main_engine import create_tree

def main():
    rclpy.init()
    node = rclpy.create_node("intermediate_test_orchestrator")
    
    # Define a single-step plan
    custom_plan = {
        "left_arm_sequence": [
            {
                "action": "PLACE",
                "target_location": "box",
                "arm": "left_arm"
            }
        ],
        "right_arm_sequence": [
            {
                "action": "PICK",
                "target_name": "target_object",
                "arm": "right_arm"
            }
        ]
    }

    # Create the official tree structure
    root = create_tree(task_description="Intermediate Robust Test")
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.visitors.append(py_trees.visitors.DisplaySnapshotVisitor())
    
    # Initialize Blackboard with the plan
    blackboard = py_trees.blackboard.Client(name="Main")
    for key in ["vlm_plan", "mission_completed", "handover_ready", "handover_starting"]:
        blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
    
    blackboard.vlm_plan = custom_plan
    blackboard.mission_completed = False
    blackboard.handover_ready = False
    blackboard.handover_starting = False

    print("🚀 Running ROBUST INTERMEDIATE Test (Main Engine Logic)...")
    
    try:
        tree.setup(timeout=15.0, node=node)
        
        print("\n--- Starting Tree Execution ---")
        # Tick until mission_completed is True
        while rclpy.ok():
            tree.tick()
            
            if getattr(blackboard, "mission_completed", False):
                print("\n✅ MISSION ACCOMPLISHED: Official termination signal received.")
                break
            
            if root.status == py_trees.common.Status.FAILURE:
                print("\n❌ MISSION FAILED: Tree returned failure status.")
                break
            
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user.")
    finally:
        print("🧹 Cleaning up...")
        tree.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
