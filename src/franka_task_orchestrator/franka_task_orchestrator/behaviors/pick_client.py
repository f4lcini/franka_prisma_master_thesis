#!/usr/bin/env python3

import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import MtcPickObject

class MtcPickActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Execute MTC Pick", action_name="/mtc_pick_object"):
        super().__init__(name=name)
        self.action_name = action_name
        
        # Variabili di stato asincrono (Futures)
        self.node = None
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None

        # Sincronizzazione Blackboard
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="target_pose", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """
        Fase di inizializzazione dell'hardware/middleware. 
        Riceve il nodo ROS 2 dall'albero radice e aggancia l'Action Server.
        """
        try:
            self.node = kwargs['node']
        except KeyError:
            raise KeyError("Il nodo ROS 2 non è stato passato al metodo setup() di py_trees.")

        self.action_client = ActionClient(self.node, MtcPickObject, self.action_name)
        self.node.get_logger().info(f"[{self.name}] Attesa dell'Action Server {self.action_name}...")
        self.action_client.wait_for_server(timeout_sec=15.0)

    def initialise(self):
        """
        Invocato istantaneamente prima del primo tick() nello stato RUNNING.
        Costruisce il payload e lo invia asincronamente.
        """
        goal_msg = MtcPickObject.Goal()
        goal_msg.target_pose = self.blackboard.target_pose
        goal_msg.approach_distance = 0.15

        self.node.get_logger().info(f"[{self.name}] Trasmissione Goal asincrona...")
        
        # Reset dei Future per permettere re-esecuzioni multiple
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.get_result_future = None

    def update(self):
        """
        Macchina a stati interna valutata ad ogni Tick Tock (es. 2 Hz).
        Mappa lo stato dei Future ROS 2 sullo stato del nodo Behavior Tree.
        """
        # Fase 1: In attesa che il server accetti/rigetti il goal
        if self.send_goal_future and not self.send_goal_future.done():
            return py_trees.common.Status.RUNNING

        # Fase 2: Goal processato, richiesta del risultato finale
        if self.send_goal_future and self.send_goal_future.done() and not self.get_result_future:
            goal_handle = self.send_goal_future.result()
            if not goal_handle.accepted:
                self.node.get_logger().error(f"[{self.name}] Rigetto matematico o logico dall'Action Server.")
                return py_trees.common.Status.FAILURE
            
            self.node.get_logger().info(f"[{self.name}] Goal accettato. Elaborazione MTC in corso...")
            self.get_result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        # Fase 3: In attesa dell'esecuzione della traiettoria fisica
        if self.get_result_future and not self.get_result_future.done():
            return py_trees.common.Status.RUNNING

        # Fase 4: Analisi del risultato computazionale
        if self.get_result_future and self.get_result_future.done():
            result = self.get_result_future.result().result
            if result.success:
                self.node.get_logger().info(f"[{self.name}] Operazione conclusa con SUCCESSO.")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().error(f"[{self.name}] Fallimento MTC: {result.message}")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        Pulizia delle risorse in caso di preemption (es. Fallback attivato).
        """
        self.send_goal_future = None
        self.get_result_future = None