#!/usr/bin/env python3

import py_trees
from geometry_msgs.msg import PoseStamped

class MockPerception(py_trees.behaviour.Behaviour):
    def __init__(self, name="Mock Perception"):
        super().__init__(name)
        # Inizializzazione del client Blackboard
        self.blackboard = py_trees.blackboard.Client(name=name)
        # Registrazione della variabile con permessi di scrittura
        self.blackboard.register_key(key="target_pose", access=py_trees.common.Access.WRITE)

    def update(self):
        pose = PoseStamped()
        pose.header.frame_id = "fr3_link0"
        
        # Posizionamento sul tavolo davanti al robot
        pose.pose.position.x = 0.45  # Distanza operativa ottimale per FR3
        pose.pose.position.y = 0.0
        # Altezza: -0.01 (livello tavolo) + 0.02 (metà cubo) = 0.01
        pose.pose.position.z = 0.01  
        
        # Orientamento: Pinza rivolta esattamente verso il basso (Grasp verticale)
        # Il quaternione [1, 0, 0, 0] inverte l'asse Z dell'end-effector verso il basso
        # Modifica temporanea per test di raggiungibilità
        pose.pose.position.x = 0.4   # Più vicino al robot (centro dello spazio di lavoro)
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.2   # 20 cm sopra la base (aria libera)
        pose.pose.orientation.w = 0.0

        self.blackboard.target_pose = pose
        return py_trees.common.Status.SUCCESS