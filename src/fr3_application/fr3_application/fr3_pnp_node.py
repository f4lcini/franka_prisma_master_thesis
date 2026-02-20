import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import (
    MotionPlanRequest, WorkspaceParameters, Constraints, 
    PositionConstraint, OrientationConstraint, JointConstraint, BoundingVolume,
    CollisionObject, AttachedCollisionObject, MoveItErrorCodes
)

class Fr3PickAndPlaceRigorous(Node):
    def __init__(self):
        super().__init__('fr3_pnp_rigorous_node')
        self.get_logger().info("Inizializzazione Client MoveGroup (Calibrazione Sub-Millimetrica)...")
        
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._scene_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')

        self.get_logger().info("Sincronizzazione stack DDS in corso...")
        self._action_client.wait_for_server()
        self._scene_client.wait_for_service()

        self.arm_group = "fr3_arm"
        self.hand_group = "hand"
        self.base_link = "fr3_link0"
        self.tcp_link = "fr3_hand_tcp"
        self.cube_name = "gazebo_cube"

        # VETTORI DI POSA RICALCOLATI SULLA GROUND TRUTH (TF2)
        # Il cubo in Gazebo è a X=0.6, base robot a X=0.1 -> Target X=0.5
        self.target_x = 0.500
        self.target_y = 0.000
        # Il centro del cubo è a Z=0.775, base robot a Z=0.760 -> Target Z=0.015
        self.target_z = 0.015 

        # Inizializzazione della topologia di collisione
        self._manage_cube_geometry(action="add")

    def _manage_cube_geometry(self, action="add"):
        obj = CollisionObject()
        obj.header.frame_id = self.base_link
        obj.id = self.cube_name

        if action == "add":
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [0.05, 0.05, 0.05]
            pose = Pose()
            pose.position.x = self.target_x
            pose.position.y = self.target_y
            pose.position.z = self.target_z 
            pose.orientation.w = 1.0
            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose)
            obj.operation = CollisionObject.ADD
        else:
            obj.operation = CollisionObject.REMOVE

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects.append(obj)
        future = self._scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def set_cube_attachment(self, attach: bool):
        att = AttachedCollisionObject()
        att.link_name = self.tcp_link
        att.object.id = self.cube_name
        att.object.operation = CollisionObject.ADD if attach else CollisionObject.REMOVE
        
        if attach: 
            # Soppressione dei check di collisione tra dita e oggetto afferrato
            att.touch_links = ["fr3_leftfinger", "fr3_rightfinger"]

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.robot_state.attached_collision_objects.append(att)
        
        future = self._scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def execute_move(self, x, y, z, precision=False):
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.num_planning_attempts = 15
        req.allowed_planning_time = 5.0
        
        # Riduzione derivate cinematiche per i micro-step di precisione
        req.max_velocity_scaling_factor = 0.1 if precision else 0.3
        req.max_acceleration_scaling_factor = 0.1 if precision else 0.3
        req.start_state.is_diff = True

        ws = WorkspaceParameters()
        ws.header.frame_id = self.base_link
        ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = -1.0, -1.0, -0.2
        ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = 1.0, 1.0, 1.5
        req.workspace_parameters = ws

        con = Constraints()
        
        # 1. Vincolo di Posizione (Sfera di Tolleranza)
        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.tcp_link
        pc.weight = 1.0
        bv = BoundingVolume()
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002 if precision else 0.01])
        target = Pose()
        target.position.x, target.position.y, target.position.z = x, y, z
        bv.primitives.append(sphere)
        bv.primitive_poses.append(target)
        pc.constraint_region = bv
        con.position_constraints.append(pc)

        # 2. Vincolo di Orientamento (Quaternione Normalizzato, TCP a piombo)
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link
        oc.link_name = self.tcp_link
        oc.orientation.x = 1.0
        oc.orientation.y = 0.0
        oc.orientation.z = 0.0
        oc.orientation.w = 0.0
        oc.weight = 1.0
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 3.14
        con.orientation_constraints.append(oc)

        req.goal_constraints.append(con)
        goal_msg.request = req

        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error("Validatore di Stato: Goal Rigettato.")
            return False
            
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        time.sleep(0.5) # Settling time per smorzamento vibrazioni fisiche
        return res_future.result().result.error_code.val == MoveItErrorCodes.SUCCESS

    def execute_gripper(self, open_grip=True):
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.hand_group
        req.start_state.is_diff = True
        con = Constraints()
        
        # Grasp Dinamico: Apertura 0.04m, Chiusura 0.023m per dito (4.6cm totali su cubo da 5cm)
        pos = 0.04 if open_grip else 0.023
        for j in ["fr3_finger_joint1", "fr3_finger_joint2"]:
            jc = JointConstraint(joint_name=j, position=pos, weight=1.0)
            # Tolleranza simmetrica per assorbimento stallo elastico
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            con.joint_constraints.append(jc)
        
        req.goal_constraints.append(con)
        goal_msg.request = req
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        res_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        # Stabilizzazione vettori attrito in Gazebo
        time.sleep(1.2) 

    def run_pick_and_place(self):
        try:
            self.execute_gripper(open_grip=True)
            
            # --- FASE 1: Avvicinamento Veloce ---
            self.get_logger().info("FASE 1: Avvicinamento in Quota")
            self.execute_move(self.target_x, self.target_y, self.target_z + 0.15)
            
            # --- FASE 2: Allineamento Assiale ---
            self.get_logger().info("FASE 2: Allineamento Assiale di Precisione")
            self.execute_move(self.target_x, self.target_y, self.target_z + 0.07, precision=True)
            
            # --- FASE 3: Discesa a Piombo ---
            self.get_logger().info("FASE 3: Discesa TCP")
            self.execute_move(self.target_x, self.target_y, self.target_z, precision=True)
            
            # --- FASE 4: Grasp Cinematico ---
            self.get_logger().info("FASE 4: Chiusura Gripper e Attacco Logico")
            self.execute_gripper(open_grip=False)
            self.set_cube_attachment(True)
            
            # --- FASE 5: Estrazione Verticale ---
            self.get_logger().info("FASE 5: Sollevamento Lento")
            self.execute_move(self.target_x, self.target_y, self.target_z + 0.15, precision=True)
            
            # --- FASE 6: Trasferimento (Place) ---
            self.get_logger().info("FASE 6: Trasferimento Asse XY")
            self.execute_move(0.400, 0.300, self.target_z + 0.15)
            self.get_logger().info("FASE 6b: Discesa per Rilascio")
            self.execute_move(0.400, 0.300, self.target_z + 0.02, precision=True)
            
            # --- FASE 7: Rilascio e Allontanamento ---
            self.get_logger().info("FASE 7: Sgancio e Allontanamento")
            self.execute_gripper(open_grip=True)
            self.set_cube_attachment(False)
            self.execute_move(0.400, 0.300, self.target_z + 0.15)
            
            self.get_logger().info("Ciclo Pick and Place Concluso Nominalmente.")

        except Exception as e:
            self.get_logger().error(f"Interruzione Critica del Task: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Fr3PickAndPlaceRigorous()
    node.run_pick_and_place()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()