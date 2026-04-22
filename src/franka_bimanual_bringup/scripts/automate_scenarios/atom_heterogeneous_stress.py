#!/usr/bin/env python3

import os
import sys
import rclpy
import py_trees
import py_trees_ros
import time
from geometry_msgs.msg import PoseStamped

# Path Setup
workspace_path = "/home/falco_robotics/vf_projects_portfolio/mm_ws/src"
if os.path.exists("/mm_ws/src"):
    workspace_path = "/mm_ws/src"

sys.path.append(os.path.join(workspace_path, "franka_bimanual_orchestrator"))
from franka_bimanual_orchestrator.behaviors.move_home_client import MoveHomeClient
from franka_bimanual_orchestrator.behaviors.pick_client import PickActionClient
from franka_bimanual_orchestrator.behaviors.place_client import PlaceActionClient

class SimpleTimer(py_trees.behaviour.Behaviour):
    def __init__(self, name="Timer", duration=3.0):
        super().__init__(name=name)
        self.duration = duration
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()
        self.logger.info(f"⏳ [{self.name}] Starting timer for {self.duration}s")

    def update(self):
        if self.start_time is None: return py_trees.common.Status.FAILURE
        elapsed = time.time() - self.start_time
        return py_trees.common.Status.SUCCESS if elapsed >= self.duration else py_trees.common.Status.RUNNING

def create_stress_tree():
    root = py_trees.composites.Parallel(
        name="Atomic_Stress_Test",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    
    # ---------------------------------------------------------
    # RIGHT ARM (HEAVY): Performs a full PICK and PLACE sequence
    # ---------------------------------------------------------
    right_seq = py_trees.composites.Sequence(name="Right_Heavy_Sequence", memory=True)
    right_pick = PickActionClient(name="Right_Heavy_Pick", prefix="right_")
    right_place = PlaceActionClient(name="Right_Heavy_Place", prefix="right_")
    right_seq.add_children([right_pick, right_place])
    
    # ---------------------------------------------------------
    # LEFT ARM (LIGHT): Performs sequence of JOINT moves and WAITS
    # ---------------------------------------------------------
    left_seq = py_trees.composites.Sequence(name="Left_Light_Sequence", memory=True)
    left_to_mid = MoveHomeClient(name="Left_to_Midway", prefix="left_")
    left_wait = SimpleTimer(name="Left_Pause_2s", duration=2.0)
    left_to_ready = MoveHomeClient(name="Left_to_Ready", prefix="left_")
    
    left_seq.add_children([left_to_mid, left_wait, left_to_ready])
    
    root.add_children([right_seq, left_seq])
    return root

def main():
    rclpy.init()
    
    # BLACKBOARD SETUP
    blackboard = py_trees.blackboard.Client(name="Atomic_Stress")
    blackboard.register_key(key="left_active_arm", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_active_arm", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="left_target_pose_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_target_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_target_location", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="vlm_plan", access=py_trees.common.Access.WRITE)
    
    # Set targets
    blackboard.left_active_arm = "left_arm"
    blackboard.right_active_arm = "right_arm"
    blackboard.right_target_name = "base_pose"  # For Pick
    blackboard.right_target_location = "shared"   # For Place
    blackboard.left_target_pose_name = "midway"
    blackboard.vlm_plan = ["stress_test"]

    # Build and run
    root = create_stress_tree()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.visitors.append(py_trees.visitors.DisplaySnapshotVisitor())
    
    print("🚀 Running ATOMIC STRESS TEST (Right=Pick, Left=MoveHome)...")
    print("RIGHT: Complex Picking at 'base_pose' (Red Cube)")
    print("LEFT:  Joint Move sequence (Midway -> Ready)")
    
    # Switch left pose
    def pre_tick_handler(tree):
        if root.children[1].children[0].status == py_trees.common.Status.SUCCESS:
             blackboard.left_target_pose_name = "ready"

    try:
        tree.setup(node_name="atomic_stress_engine", timeout=15.0)
        
        # We use a managed loop to avoid duplicate spinning of the same node/handles
        while rclpy.ok():
            pre_tick_handler(tree)
            
            # Tick the tree once
            tree.tick()
            
            # Exit conditions
            if root.status == py_trees.common.Status.SUCCESS:
                print("\n✅ STRESS TEST COMPLETED SUCCESSFULLY!")
                break
            if root.status == py_trees.common.Status.FAILURE:
                print("\n❌ STRESS TEST FAILED.")
                break
            
            # Spin to allow ROS 2 communication to process
            rclpy.spin_once(tree.node, timeout_sec=0.1)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n🛑 Stopped.")
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
