import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import MtcPlaceObject
import copy

class MtcPlaceActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Execute MTC Place", action_name="/mtc_place_object"):
        super().__init__(name=name)
        self.action_name = action_name
        self.node = None
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="target_pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="active_arm", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_location", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError:
            self.logger.error(f"[{self.name}] ROS 2 node not found in setup kwargs. Cannot create Action Client!")
            return False

        self.action_client = ActionClient(self.node, MtcPlaceObject, self.action_name)
        self.logger.info(f"[{self.name}] Waiting for MTC Place server at '{self.action_name}'...")
        self.action_client.wait_for_server(timeout_sec=1.0)
        return True

    def initialise(self):
        self.logger.info(f"[{self.name}] Initializing MTC Place...")
        self.send_goal_future = None
        self.get_result_future = None
        
        target_arm = "any"
        target_loc = None
        if hasattr(self.blackboard, "active_arm"):
            target_arm = self.blackboard.active_arm
        if hasattr(self.blackboard, "target_location"):
            target_loc = self.blackboard.target_location

        from geometry_msgs.msg import PoseStamped
        target_pose = None
        
        # ZONE ASTRATTE (HARDCODED): Non usiamo YOLO se la destinazione è lo spazio neutro
        if target_loc == "shared_workspace":
            self.logger.info(f"[{self.name}] Target is a known abstract zone: shared_workspace.")
            target_pose = PoseStamped()
            target_pose.header.frame_id = "world"
            # TODO: Placeholder Simulation vs Hardware Absolute Positions
            target_pose.pose.position.x = 0.50 # Center of the table Forward
            target_pose.pose.position.y = 0.00 # Exact middle line between the 2 arms
            target_pose.pose.position.z = 0.15 # Just above table surface
            target_pose.pose.orientation.w = 1.0
        else:
            try:
                target_pose = self.blackboard.target_pose
                if target_pose is None:
                    raise ValueError("Target pose is None from Perception")
            except AttributeError:
                self.logger.error(f"[{self.name}] No dynamic 'target_pose' found on the blackboard!")
                self.status = py_trees.common.Status.FAILURE
                return

        goal_msg = MtcPlaceObject.Goal()
        goal_msg.arm = target_arm
        goal_msg.place_pose = copy.deepcopy(target_pose)
        goal_msg.retreat_distance = 0.15 

        if not self.action_client.server_is_ready():
            self.logger.error(f"[{self.name}] Action server {self.action_name} is OFF!")
            self.status = py_trees.common.Status.FAILURE
            return

        self.logger.info(f"[{self.name}] Sending action goal asynchronously...")
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.status = py_trees.common.Status.RUNNING

    def update(self):
        if hasattr(self, 'status') and self.status == py_trees.common.Status.FAILURE:
            return py_trees.common.Status.FAILURE

        if self.get_result_future is None:
            if self.send_goal_future and self.send_goal_future.done():
                goal_handle = self.send_goal_future.result()
                if not goal_handle.accepted:
                    self.logger.error(f"[{self.name}] MTC Place goal REJECTED by server!")
                    return py_trees.common.Status.FAILURE
                self.logger.info(f"[{self.name}] MTC Place goal ACCEPTED. Waiting for execution...")
                self.get_result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING
        
        if self.get_result_future.done():
            result = self.get_result_future.result().result
            if result.success:
                self.logger.info(f"[{self.name}] MTC Place SUCCEEDED: {result.message}")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"[{self.name}] MTC Place FAILED: {result.message}")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.send_goal_future = None
        self.get_result_future = None
