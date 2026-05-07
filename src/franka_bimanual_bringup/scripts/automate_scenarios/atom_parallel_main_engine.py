#!/usr/bin/env python3

import rclpy
import py_trees
import py_trees_ros
import sys
import os

# Import main_engine logic
from franka_bimanual_orchestrator.main_engine import create_tree

def main():
    rclpy.init(args=sys.argv)
    
    custom_plan = {
        "left_arm_sequence": [
            {
                "action": "PLACE",
                "target_location": "box",
                "arm": "left_arm"
            },
            {
                "action": "MOVE_HOME",
                "arm": "left_arm"
            }
        ],
        "right_arm_sequence": [
            {
                "action": "PICK",
                "target_name": "target_object",
                "arm": "right_arm"
            },
            {
                "action": "PLACE",
                "target_location": "shared",
                "arm": "right_arm"
            }
        ]
    }

    # Create the standard tree
    root = create_tree(task_description="Custom Parallel Test")
    
    # Initialize the ROS-integrated tree
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    
    # Add display visitor for debugging
    tree.visitors.append(py_trees.visitors.DisplaySnapshotVisitor())
    
    try:
        print("🔧 Setting up Behavior Tree...")
        tree.setup(node_name="atom_parallel_engine", timeout=15.0)
    except Exception as e:
        print(f"❌ Setup failed: {e}")
        return

    # PRE-INJECT THE PLAN INTO BLACKBOARD
    blackboard = py_trees.blackboard.Client(name="Main")
    blackboard.register_key(key="vlm_plan", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="mission_completed", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="handover_ready", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="handover_starting", access=py_trees.common.Access.WRITE)
    
    blackboard.vlm_plan = custom_plan
    blackboard.mission_completed = False
    blackboard.handover_ready = False
    blackboard.handover_starting = False

    print("\n🚀 [ATOM] Starting Parallel Main Engine Test...")
    print("Plan injected. Bypassing Gemini VLM.")
    
    try:
        # Tick at 1Hz
        tree.tick_tock(period_ms=1000)
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user.")
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
