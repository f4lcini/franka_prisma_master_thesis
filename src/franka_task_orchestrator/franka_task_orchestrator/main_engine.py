#!/usr/bin/env python3

"""
================================================================================
Author: Falco Robotics 
Code Description: 
This is the central execution framework (Main Engine) employing a Behavior Tree 
(BT) to orchestrate tasks across the robotic cell. It uses py_trees and py_trees_ros 
to build a Directed Acyclic Graph (DAG) that coordinates High-Level perception (VLM) 
with Low-Level execution (MoveIt Task Constructor or direct Cartesians).

Pipeline: Task Orchestration & Mission Control

Implementation Steps Summary:
- NODE & DEPENDENCY SETUP (Steps 1-2): Import the py_trees architecture and internal behavior modules (MockPerception, PickClient).
- TREE CONSTRUCTION (Steps 3-4): Create a root Sequence with Memory to build a strict execution loop that saves completed behavior states.
- BEHAVIOR INSTANTIATION (Step 5): Initialize the specific Behavior Leaf nodes (e.g., Simulate_Vision_Layer, MTC_Hardware_Execution).
- TOPOLOGY LINKING (Step 6): Attach the initialized behaviors as children to the root DAG Sequence.
- ROS 2 WRAPPER INITIALIZATION (Steps 7-8): Launch the main function and securely wrap the py_trees object inside a py_trees_ros BehaviourTree node.
- SYSTEM HANDSHAKE (Steps 9-10): Call tree.setup() with a rigorous Timeout to ensure all external Action Servers (MTC, VLM) are reachable before starting.
- ASYNCHRONOUS TICK LOOP (Step 11): Run the node using tick_tock, triggering cyclic tree executions (e.g., 2 Hz) until completion or SIGINT.
- GRACEFUL SHUTDOWN (Step 12): Systematically shutdown the tree and drop rclpy loops when the process terminates.
================================================================================
"""

# Step 1-2: Import the py_trees architecture and internal behavior modules (MockPerception, PickClient).
import rclpy
import py_trees
import py_trees_ros
import sys

from franka_task_orchestrator.behaviors.perception_mock import MockPerception
from franka_task_orchestrator.behaviors.pick_client import MtcPickActionClient

def create_tree():
    """Costruisce la topologia del grafo diretto aciclico (DAG)."""
    # Step 3-4: Create a root Sequence with Memory to build a strict execution loop that saves completed behavior states.
    root = py_trees.composites.Sequence(name="Pick_and_Place_MVP", memory=True)
    
    # Step 5: Initialize the specific Behavior Leaf nodes (e.g., Simulate_Vision_Layer, MTC_Hardware_Execution).
    mock_perception = MockPerception(name="Simulate_Vision_Layer")
    pick_action = MtcPickActionClient(name="MTC_Hardware_Execution")
    
    # Step 6: Attach the initialized behaviors as children to the root DAG Sequence.
    root.add_children([mock_perception, pick_action])
    return root

def main():
    # Step 7: Launch the main function (rclpy init).
    rclpy.init(args=sys.argv)
    
    # Costruzione dell'albero
    root = create_tree()
    
    # Step 8: Securely wrap the py_trees object inside a py_trees_ros BehaviourTree node.
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    
    try:
        # Step 9-10: Call tree.setup() with a rigorous Timeout to ensure all external Action Servers are reachable before starting.
        tree.setup(node_name="task_orchestrator_engine", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        print(f"Errore critico: {e}. Verifica che il server mtc_pick_server sia in esecuzione.")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    print("\n--- Inizializzazione Orchestratore completata. Avvio Tick Loop (2 Hz) ---\n")
    
    try:
        # Step 11: Run the node using tick_tock, triggering cyclic tree executions (e.g., 2 Hz) until completion or SIGINT.
        tree.tick_tock(period_ms=500)
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        print("Interruzione richiesta dall'utente (SIGINT).")
    finally:
        # Step 12: Systematically shutdown the tree and drop rclpy loops when the process terminates.
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()