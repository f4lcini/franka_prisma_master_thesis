#!/usr/bin/env python3

import os
import sys
import rclpy
import py_trees
import py_trees_ros.trees
from geometry_msgs.msg import PoseStamped

# Path Setup
workspace_path = "/home/falco_robotics/vf_projects_portfolio/mm_ws/src"
if os.path.exists("/mm_ws/src"):
    workspace_path = "/mm_ws/src"

sys.path.append(os.path.join(workspace_path, "franka_bimanual_orchestrator"))

from franka_bimanual_orchestrator.behaviors.pick_client import PickActionClient
from franka_bimanual_orchestrator.behaviors.place_client import PlaceActionClient
from franka_bimanual_orchestrator.behaviors.give_client import GiveActionClient
from franka_bimanual_orchestrator.behaviors.take_client import TakeActionClient
from franka_bimanual_orchestrator.behaviors.move_home_client import MoveHomeClient

def create_handshake_tree():
    bimanual_handshake = py_trees.composites.Parallel(
        name="Bimanual_Handshake_Scenario",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    
    # ------------------------------------------------------
    # RIGHT ARM: DONOR
    # ------------------------------------------------------
    # 1. Right Pick (at target_object)
    right_set_pick = py_trees.behaviours.SetBlackboardVariable(name="RT_Set_Pick_Tgt", variable_name="right_target_name", variable_value="target_object", overwrite=True)
    right_pick = PickActionClient(name="Right_Pick_Cube", action_name="/pick_object", prefix="right_")
    
    # 2. Right Prepare for Handover (at shared)
    right_set_give = py_trees.behaviours.SetBlackboardVariable(name="RT_Set_Give_Tgt", variable_name="right_target_name", variable_value="shared", overwrite=True)
    rt_signal = py_trees.behaviours.SetBlackboardVariable(name="Right_Signal_Ready", variable_name="left_sync_flag", variable_value=True, overwrite=True)
    rt_wait = py_trees.behaviours.WaitForBlackboardVariable(name="Wait_For_Left", variable_name="right_sync_flag")
    
    # 3. Right Give
    right_give = GiveActionClient(name="Right_Give_MidAir", action_name="/give_object", prefix="right_")
    
    # 4. Right Home
    right_home = MoveHomeClient(name="Right_Back_Home", action_name="/move_home", prefix="right_")
    
    right_seq.add_children([right_set_pick, right_pick, right_set_give, rt_signal, rt_wait, right_give, right_home])

    # ------------------------------------------------------
    # LEFT ARM: RECIPIENT
    # ------------------------------------------------------
    left_seq = py_trees.composites.Sequence(name="Left_Recipient_Sequence", memory=True)
    
    # 1. Left Setup (Go midway first)
    left_setup = MoveHomeClient(name="Left_Wait_Midway", action_name="/move_home", prefix="left_")
    
    # 2. Left Prepare for Handover (at shared)
    left_set_take = py_trees.behaviours.SetBlackboardVariable(name="LT_Set_Take_Tgt", variable_name="left_target_name", variable_value="shared", overwrite=True)
    lt_signal = py_trees.behaviours.SetBlackboardVariable(name="Left_Signal_Ready", variable_name="right_sync_flag", variable_value=True, overwrite=True)
    lt_wait = py_trees.behaviours.WaitForBlackboardVariable(name="Wait_For_Right", variable_name="left_sync_flag")
    
    # 3. Left Take from donor
    left_take = TakeActionClient(name="Left_Take_MidAir", action_name="/take_object", prefix="left_")
    
    # 4. Left Place (at box)
    left_set_place = py_trees.behaviours.SetBlackboardVariable(name="LT_Set_Place_Tgt", variable_name="left_target_name", variable_value="box", overwrite=True)
    left_place = PlaceActionClient(name="Left_Place_Box", action_name="/place_object", prefix="left_")
    
    left_seq.add_children([left_setup, left_set_take, lt_signal, lt_wait, left_take, left_set_place, left_place])

    bimanual_handshake.add_children([right_seq, left_seq])
    return bimanual_handshake

def main():
    rclpy.init()
    node = rclpy.create_node("bimanual_handshake_orchestrator")
    
    # Initialize Blackboard
    blackboard = py_trees.blackboard.Client()
    blackboard.register_key(key="left_active_arm", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_active_arm", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="left_target_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_target_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="left_target_pose", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_target_pose", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_target_location", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="left_target_location", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="left_target_pose_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_target_pose_name", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="vlm_plan", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="handover_ready", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="handover_starting", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="handover_target_pose", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="right_at_rendezvous", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="left_at_rendezvous", access=py_trees.common.Access.WRITE)

    # Initialize mandatory keys for Strategic behaviors
    blackboard.vlm_plan = "manual_handshake_demo"
    blackboard.handover_ready = False
    blackboard.handover_starting = False
    blackboard.right_at_rendezvous = False
    blackboard.left_at_rendezvous = False

    # Set Handover Targets
    blackboard.left_active_arm = "left_arm"
    blackboard.right_active_arm = "right_arm"
    
    # ----------------------------------------------------
    # PICK SETUP: Right arm targets 'target_object'
    # ----------------------------------------------------
    blackboard.right_target_name = "target_object" 
    blackboard.right_target_pose = None
    
    # ----------------------------------------------------
    # HANDOVER SETUP: Shared zone transfer
    # ----------------------------------------------------
    blackboard.left_target_name = "shared"
    blackboard.right_target_name = "shared" 
    blackboard.handover_target_pose = None 

    # ----------------------------------------------------
    # FINAL DESTINATION
    # ----------------------------------------------------
    blackboard.left_target_location = "box"
    
    # Setup midway poses
    blackboard.left_target_pose_name = "midway"
    blackboard.right_target_pose_name = "midway"

    # Create and setup tree
    root = create_handshake_tree()
    tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=True)
    tree.setup(timeout=15.0, node=node)
    
    print("\n🚀 Starting Coordinated Bimanual Handshake Scenario...")
    print("-------------------------------------------------------")
    print("Layer: Skill Handshaking (Give/Take Mutex Validation)")
    
    try:
        while rclpy.ok():
            tree.tick()
            if root.status == py_trees.common.Status.SUCCESS:
                print("\n✅ HANDSHAKE COMPLETED SUCCESSFULLY!")
                break
            if root.status == py_trees.common.Status.FAILURE:
                print("\n❌ HANDSHAKE FAILED.")
                break
            time_to_sleep = 0.5
            rclpy.spin_once(node, timeout_sec=time_to_sleep)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
