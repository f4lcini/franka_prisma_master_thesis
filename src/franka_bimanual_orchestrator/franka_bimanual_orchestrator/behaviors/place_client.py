import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import PlaceObject
import copy
from geometry_msgs.msg import PoseStamped

class PlaceActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Execute Place", action_name="/place_object", prefix="left_"):
        super().__init__(name=name)
        self.action_name = action_name
        self.prefix = prefix
        self.node = None
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=f"{prefix}target_pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"{prefix}active_arm", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"{prefix}target_location", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="handover_ready", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="handover_starting", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError:
            self.logger.error(f"[{self.name}] ROS 2 node not found in setup kwargs!")
            return False

        self.action_client = ActionClient(self.node, PlaceObject, self.action_name)
        self.logger.info(f"[{self.name}] Waiting for Place server...")
        self.action_client.wait_for_server(timeout_sec=1.0)
        return True

    def initialise(self):
        self.logger.info(f"[{self.name}] Initializing Place...")
        self.send_goal_future = None
        self.get_result_future = None
        
        target_arm = "any"
        target_loc = "none"
        if hasattr(self.blackboard, f"{self.prefix}active_arm"):
            target_arm = getattr(self.blackboard, f"{self.prefix}active_arm")
        if hasattr(self.blackboard, f"{self.prefix}target_location"):
            target_loc = getattr(self.blackboard, f"{self.prefix}target_location")

        # PREDEFINED TARGET SUPPORT
        if target_loc in ["base_pose", "shared", "box"]:
            self.logger.info(f"[{self.name}] Using predefined target location: '{target_loc}'")
            target_pose = PoseStamped()
            target_pose.header.frame_id = target_loc
            target_pose.header.stamp = self.node.get_clock().now().to_msg()
        else:
            try:
                target_pose = getattr(self.blackboard, f"{self.prefix}target_pose")
            except AttributeError:
                self.logger.error(f"[{self.name}] No dynamic '{self.prefix}target_pose' found!")
                self.status = py_trees.common.Status.FAILURE
                return

        goal_msg = PlaceObject.Goal()
        goal_msg.arm = target_arm
        goal_msg.place_pose = copy.deepcopy(target_pose)
        
        # INCREASE RETREAT FOR HANDOVERS: Give more space to the other arm
        if target_loc == "shared":
            goal_msg.retreat_distance = 0.40
        else:
            goal_msg.retreat_distance = 0.15 

        self.logger.info(f"[{self.name}] Sending Place Goal for {target_arm} to {goal_msg.place_pose.header.frame_id}")
        if target_loc == "shared":
             self.blackboard.handover_starting = True
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.status = py_trees.common.Status.RUNNING

    def update(self):
        if hasattr(self, 'status') and self.status == py_trees.common.Status.FAILURE:
            return py_trees.common.Status.FAILURE

        if self.get_result_future is None:
            if self.send_goal_future and self.send_goal_future.done():
                goal_handle = self.send_goal_future.result()
                if not goal_handle.accepted:
                    return py_trees.common.Status.FAILURE
                self.get_result_future = goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING
        
        if self.get_result_future.done():
            result = self.get_result_future.result().result
            if result.success:
                target_loc = getattr(self.blackboard, f"{self.prefix}target_location", "none")
                if target_loc == "shared":
                    self.blackboard.handover_ready = True
                    self.logger.info(f"[{self.name}] Shared Placement logic complete. Signalling Handover.")
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.send_goal_future = None
        self.get_result_future = None
