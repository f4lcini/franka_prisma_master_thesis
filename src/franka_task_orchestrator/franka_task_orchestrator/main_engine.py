#!/usr/bin/env python3

import rclpy
import py_trees
import py_trees_ros
import sys

from franka_task_orchestrator.behaviors.perception_mock import MockPerception
from franka_task_orchestrator.behaviors.pick_client import MtcPickActionClient

def create_tree():
    """Costruisce la topologia del grafo diretto aciclico (DAG)."""
    # Il nodo Sequence con memory=True ricorda i figli già completati
    root = py_trees.composites.Sequence(name="Pick_and_Place_MVP", memory=True)
    
    mock_perception = MockPerception(name="Simulate_Vision_Layer")
    pick_action = MtcPickActionClient(name="MTC_Hardware_Execution")
    
    # Aggancio topologico
    root.add_children([mock_perception, pick_action])
    return root

def main():
    rclpy.init(args=sys.argv)
    
    # Costruzione dell'albero
    root = create_tree()
    
    # Istanziazione del wrapper ROS 2 per il Behavior Tree
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    
    try:
        # Il timeout di setup assicura che l'Action Server MTC sia online prima di iniziare
        tree.setup(node_name="task_orchestrator_engine", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        print(f"Errore critico: {e}. Verifica che il server mtc_pick_server sia in esecuzione.")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    print("\n--- Inizializzazione Orchestratore completata. Avvio Tick Loop (2 Hz) ---\n")
    
    try:
        # Avvio del loop asincrono (500 ms = 2 Hz)
        tree.tick_tock(period_ms=500)
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        print("Interruzione richiesta dall'utente (SIGINT).")
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()