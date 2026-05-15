import py_trees
import rclpy
from rclpy.action import ActionClient
from franka_custom_interfaces.action import PlaceObject
import copy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class PlaceActionClient(py_trees.behaviour.Behaviour):
    def __init__(self, name="Execute Place", action_name="/place_object", prefix="left_", target_location=None):
        super().__init__(name=name)
        self.action_name = action_name
        self.prefix = prefix
        self.target_loc_override = target_location
        self.node = None
        # ... rest of init ...
        self.action_client = None
        self.send_goal_future = None
        self.get_result_future = None
        self.lock_pub = None
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key=f"{prefix}target_pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"{prefix}active_arm", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"{prefix}target_location", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="handover_ready", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="handover_starting", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="mission_type", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="mission_metadata", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError:
            self.logger.error(f"[{self.name}] ROS 2 node not found in setup kwargs!")
            return False

        self.action_client = ActionClient(self.node, PlaceObject, self.action_name)
        self.lock_pub = self.node.create_publisher(String, "/franka/shared_zone/lock", 10)
        self.logger.info(f"[{self.name}] Waiting for Place server...")
        self.action_client.wait_for_server(timeout_sec=1.0)
        return True

    def initialise(self):
        self.logger.info(f"[{self.name}] Initializing Place...")
        self.send_goal_future = None
        self.get_result_future = None
        
        target_arm = "any"
        target_loc = "none"

        # Priorità: Override da costruttore -> Blackboard
        if self.target_loc_override:
            target_loc = self.target_loc_override
        elif hasattr(self.blackboard, f"{self.prefix}target_location"):
            target_loc = getattr(self.blackboard, f"{self.prefix}target_location")
        
        if hasattr(self.blackboard, f"{self.prefix}active_arm"):
            target_arm = getattr(self.blackboard, f"{self.prefix}active_arm")

        # PREDEFINED TARGET SUPPORT
        if target_loc in ["base_pose", "shared", "box", "target_object", "box_ws_sx", "box_ws_dx"]:
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
        
        # --- ROBUSTNESS: Standard retreat, rely on MOVE_HOME for clearing zones ---
        goal_msg.retreat_distance = 0.10

        self.logger.info(f"[{self.name}] Sending Place Goal for {target_arm} to {goal_msg.place_pose.header.frame_id}")
        metadata = getattr(self.blackboard, "mission_metadata", {})
        if target_loc == metadata.get("shared_zone", "shared"):
            self.blackboard.handover_starting = True
            lock_msg = String()
            lock_msg.data = f"{self.prefix.upper()}ARM_OCCUPIED"
            self.lock_pub.publish(lock_msg)
            self.logger.info(f"🔒 [{self.name}] SHARED ZONE LOCKED by {self.prefix}arm")

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
                mission_type = getattr(self.blackboard, "mission_type", "SIMPLE")
                metadata = getattr(self.blackboard, "mission_metadata", {})
                target_loc = getattr(self.blackboard, f"{self.prefix}target_location", "none")
                
                # Dynamic Coordination: Check if this place is part of a handover
                is_handover = (mission_type == "HANDOVER")
                is_shared = (target_loc == metadata.get("shared_zone", "shared"))

                if is_handover and is_shared:
                    # DIRECT WRITE TO BLACKBOARD
                    self.blackboard.handover_ready = True
                    
                    # ROS 2 LOCK RELEASE
                    lock_msg = String()
                    lock_msg.data = f"{self.prefix.upper()}ARM_RELEASED"
                    self.lock_pub.publish(lock_msg)
                    
                    self.logger.info(f"🚩🚩🚩 [{self.name}] MISSION-BASED HANDOVER_READY SIGNAL SET! 🚩🚩🚩")
                    self.logger.info(f"🔓 [{self.name}] SHARED ZONE RELEASED by {self.prefix}arm")
                return py_trees.common.Status.SUCCESS
            
            # --- Robustness: Return FAILURE instead of infinite retry ---
            error_msg = result.message.lower() if hasattr(result, 'message') else ""
            self.logger.error(f"[{self.name}] Place FAILED: {error_msg}")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.send_goal_future = None
        self.get_result_future = None
