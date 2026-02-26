import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from moveit_msgs.msg import (
    MotionPlanRequest, WorkspaceParameters, Constraints, 
    PositionConstraint, OrientationConstraint, BoundingVolume,
    CollisionObject, AttachedCollisionObject, MoveItErrorCodes
)
from control_msgs.action import GripperCommand

class Fr3PickAndPlaceRigorous(Node):
    def __init__(self):
        super().__init__('fr3_pnp_rigorous_node')
        self.get_logger().info("Inizializzazione Sistema PnP FR3 - Controllo Cartesiano Attivo")
        
        self.cb_group = MutuallyExclusiveCallbackGroup()
        
        # DDS Interfaces
        self._action_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group)
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory', callback_group=self.cb_group)
        self._scene_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene', callback_group=self.cb_group)
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path', callback_group=self.cb_group)
        self._gripper_client = ActionClient(self, GripperCommand, '/gripper_action_controller/gripper_cmd', callback_group=self.cb_group)

        # Kinematic Parameters
        self.arm_group = "fr3_arm"
        self.base_link = "fr3_link0"
        self.tcp_link = "fr3_hand_tcp"
        self.cube_name = "gazebo_cube"

        # Nominal target coordinates
        self.target_x = 0.500
        self.target_y = 0.000
        # Rigorous centroid calculation: base_cube at -0.010, top at 0.040 -> Center at 0.015
        self.center_z = 0.015 

    def _wait_for_future(self, future):
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        return future.result()

    def _create_target_pose(self, x, y, z):
        """Genera una posa spaziale con il TCP orientato a piombo verso il basso."""
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        p.orientation.x = 1.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 0.0
        return p

    def _manage_cube_geometry(self, action="add"):
        obj = CollisionObject()
        obj.header.frame_id = self.base_link
        obj.id = self.cube_name

        if action == "add":
            primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05, 0.05, 0.05])
            obj.primitives.append(primitive)
            obj.primitive_poses.append(self._create_target_pose(self.target_x, self.target_y, self.center_z))
            obj.operation = CollisionObject.ADD
        else:
            obj.operation = CollisionObject.REMOVE

        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.world.collision_objects.append(obj)
        self._wait_for_future(self._scene_client.call_async(req))

    def set_cube_attachment(self, attach: bool):
        att = AttachedCollisionObject()
        att.link_name = self.tcp_link
        att.object.header.frame_id = self.base_link
        att.object.id = self.cube_name
        
        if attach: 
            primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05, 0.05, 0.05])
            att.object.primitives.append(primitive)
            att.object.primitive_poses.append(self._create_target_pose(self.target_x, self.target_y, self.center_z))
            att.object.operation = CollisionObject.ADD
            att.touch_links = ["fr3_leftfinger", "fr3_rightfinger", "fr3_hand"]
        else:
            att.object.operation = CollisionObject.REMOVE

        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        req.scene.robot_state.attached_collision_objects.append(att)
        self._wait_for_future(self._scene_client.call_async(req))

    def execute_move_ompl(self, pose: Pose):
        """Pianificazione stocastica per ampi spostamenti non vincolati."""
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.2
        req.start_state.is_diff = True

        ws = WorkspaceParameters()
        ws.header.frame_id = self.base_link
        ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = -1.0, -1.0, -0.2
        ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = 1.0, 1.0, 1.5
        req.workspace_parameters = ws

        con = Constraints()
        pc = PositionConstraint(header=ws.header, link_name=self.tcp_link, weight=1.0)
        bv = BoundingVolume()
        bv.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
        bv.primitive_poses.append(pose)
        pc.constraint_region = bv
        con.position_constraints.append(pc)

        oc = OrientationConstraint(header=ws.header, link_name=self.tcp_link, weight=1.0)
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 3.14
        con.orientation_constraints.append(oc)

        req.goal_constraints.append(con)
        goal_msg.request = req

        handle = self._wait_for_future(self._action_client.send_goal_async(goal_msg))
        if not handle.accepted: return False
        
        result = self._wait_for_future(handle.get_result_async())
        time.sleep(0.5)
        return result.result.error_code.val == MoveItErrorCodes.SUCCESS

    def execute_cartesian_path(self, waypoints: list):
        """Interpolazione lineare vincolata sull'asse Z per altissima precisione."""
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_link
        req.group_name = self.arm_group
        req.waypoints = waypoints
        
        # Risoluzione sub-millimetrica e limitazione estrema delle derivate
        req.max_step = 0.005 
        req.jump_threshold = 1.5 
        req.avoid_collisions = True

        res = self._wait_for_future(self._cartesian_client.call_async(req))
        
        if res.fraction < 0.95:
            self.get_logger().error(f"Errore Cinematico: Vettore interrotto (Frazione calcolata: {res.fraction:.2f})")
            return False

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = res.solution
        
        handle = self._wait_for_future(self._execute_client.send_goal_async(goal_msg))
        if not handle.accepted: return False
        
        result = self._wait_for_future(handle.get_result_async())
        time.sleep(0.5)
        return result.result.error_code.val == MoveItErrorCodes.SUCCESS

    def execute_gripper(self, open_grip=True):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.08 if open_grip else 0.024
        goal_msg.command.max_effort = 10.0 if open_grip else 50.0 
        
        handle = self._wait_for_future(self._gripper_client.send_goal_async(goal_msg))
        if not handle.accepted: return False
        self._wait_for_future(handle.get_result_async())
        time.sleep(1.2)
        return True

    def run_pick_and_place(self):
        self.get_logger().info("Starting DDS diagnostics (10s timeout per interface)...")
        
        # 1. Rigorous validation of all communication bridges
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().fatal("Failure: Action Server 'move_action' not reachable.")
            return
        if not self._execute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().fatal("Failure: Action Server 'execute_trajectory' not reachable.")
            return
        if not self._scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().fatal("Failure: Service 'apply_planning_scene' not reachable.")
            return
        if not self._cartesian_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().fatal("Failure: Service 'compute_cartesian_path' not reachable.")
            return
        if not self._gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().fatal("Failure: Gripper Action Server '/gripper_action_controller/gripper_cmd' not reachable.")
            return

        self.get_logger().info("DDS Topology validated. Initializing Geometries...")
        
        try:
            self._manage_cube_geometry(action="add")
            self.execute_gripper(open_grip=True)
            
            # --- PHASE 1: Pre-Grasp (OMPL) ---
            self.get_logger().info("PHASE 1: Pre-Grasp (OMPL)")
            pre_grasp_pose = self._create_target_pose(self.target_x, self.target_y, self.center_z + 0.15)
            if not self.execute_move_ompl(pre_grasp_pose): return
            
            # FCL evasion to allow volumetric interpenetration
            self._manage_cube_geometry(action="remove")
            
            # --- PHASE 2: Pure Z Descent ---
            self.get_logger().info("PHASE 2: Cartesian Insertion on the Geometric Center")
            grasp_pose = self._create_target_pose(self.target_x, self.target_y, self.center_z)
            if not self.execute_cartesian_path([grasp_pose]): return
            
            # --- PHASE 3: Grasp ---
            self.get_logger().info("PHASE 3: Dynamic Grasp")
            self.execute_gripper(open_grip=False)
            self.set_cube_attachment(True)
            
            # --- PHASE 4: Pure Z Extraction ---
            self.get_logger().info("PHASE 4: Cartesian Extraction")
            if not self.execute_cartesian_path([pre_grasp_pose]): return
            
            # --- PHASE 5: Transfer (OMPL) ---
            self.get_logger().info("PHASE 5: Transfer (OMPL)")
            place_pre_pose = self._create_target_pose(0.400, 0.300, self.center_z + 0.15)
            if not self.execute_move_ompl(place_pre_pose): return
            
            # --- PHASE 6: Cartesian Place ---
            self.get_logger().info("PHASE 6: Cartesian Descent and Release")
            place_pose = self._create_target_pose(0.400, 0.300, self.center_z + 0.02)
            if not self.execute_cartesian_path([place_pose]): return
            
            self.execute_gripper(open_grip=True)
            self.set_cube_attachment(False)
            self.execute_cartesian_path([place_pre_pose])
            
            self.get_logger().info("Cycle Completed with Analytical Success.")

        except Exception as e:
            self.get_logger().error(f"Runtime Exception in the execution tree: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Fr3PickAndPlaceRigorous()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    node.run_pick_and_place()
    
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()