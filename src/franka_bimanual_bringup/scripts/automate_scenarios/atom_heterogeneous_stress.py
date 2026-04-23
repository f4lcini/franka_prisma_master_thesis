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
from franka_bimanual_orchestrator.behaviors.wait_client import WaitActionClient

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
    """
    Sequential Task with Parallel Execution (Independent Arms):
    - Root: Parallel structure allows both arms to be active simultaneously.
    - Sync: Done via 'shared' workspace logic in Pick/Place nodes.
    
    RIGHT: Pick(base_pose) -> Place(shared)
    LEFT:  Pick(shared) -> Place(box)
    """
    root = py_trees.composites.Parallel(
        name="Parallel_Handover_Stress",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    
    # RIGHT ARM SEQUENCE
    right_seq = py_trees.composites.Sequence(name="Right_Arm_Sequence", memory=True)
    right_pick = PickActionClient(name="Right_Pick_Base", prefix="right_")
    right_place = PlaceActionClient(name="Right_Place_Shared", prefix="right_")
    right_seq.add_children([right_pick, right_place])
    
    # LEFT ARM SEQUENCE (FLUID HANDOVER)
    # 1. Move to Midway + 2. Wait for Release Topic -> 3. Start Picking
    left_seq = py_trees.composites.Sequence(name="Left_Arm_Sequence", memory=True)
    left_to_mid = MoveHomeClient(name="Left_to_Midway", prefix="left_")
    left_wait = WaitActionClient(name="Wait_for_Right_Release", prefix="left_")
    left_pick = PickActionClient(name="Left_Pick_Shared", prefix="left_")
    left_place = PlaceActionClient(name="Left_Place_Box", prefix="left_")
    left_seq.add_children([left_to_mid, left_wait, left_pick, left_place])
    
    root.add_children([right_seq, left_seq])
    return root

def main():
    rclpy.init()
    
    # BLACKBOARD SETUP
    blackboard = py_trees.blackboard.Client(name="Atomic_Stress")
    blackboard.register_key(key="left_active_arm", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_active_arm", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="left_target_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_target_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="left_target_location", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_target_location", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="left_target_pose_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="vlm_plan", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="handover_starting", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="handover_ready", access=py_trees.common.Access.WRITE)
    
    # Set targets for sequential bimanual handover
    blackboard.left_active_arm = "left_arm"
    blackboard.right_active_arm = "right_arm"
    blackboard.left_target_pose_name = "midway"
    
    blackboard.handover_starting = False
    blackboard.handover_ready = False
    
    blackboard.right_target_name = "base_pose"  # Right Arm pick
    blackboard.right_target_location = "shared"   # Right Arm place
    
    blackboard.left_target_name = "shared"     # Left Arm pick
    blackboard.left_target_location = "box"      # Left Arm place
    
    blackboard.vlm_plan = ["handover_sequence"]

    # Build and run
    root = create_stress_tree()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.visitors.append(py_trees.visitors.DisplaySnapshotVisitor())
    
    print("🚀 Running SEQUENTIAL HANDOVER TEST...")
    print("1. RIGHT: Pick at 'base_pose'")
    print("2. RIGHT: Place at 'shared'")
    print("3. LEFT:  Pick at 'shared'")
    print("4. LEFT:  Place at 'box'")

    try:
        tree.setup(node_name="atomic_stress_engine", timeout=15.0)
        
        while rclpy.ok():
            # Tick the tree once
            tree.tick()
            
            # Exit conditions
            if root.status == py_trees.common.Status.SUCCESS:
                print("\n✅ SEQUENTIAL HANDOVER COMPLETED SUCCESSFULLY!")
                break
            if root.status == py_trees.common.Status.FAILURE:
                print("\n❌ SEQUENTIAL HANDOVER FAILED.")
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
