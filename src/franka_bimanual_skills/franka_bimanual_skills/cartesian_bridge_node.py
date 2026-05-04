"""
cartesian_bridge_node.py
========================
Thin bridge that replaces the JTC execution path.

Architecture:
  Skills → ParallelMove action → [THIS NODE]
                                     |
                          MoveIt plan (joint space)
                                     |
                          FK → Cartesian waypoints
                                     |
                          Stream at 20Hz to:
                            /cartesian_impedance_left/commands
                            /cartesian_impedance_right/commands
                                     |
                          franka_ros2_multimanual (CartesianImpedanceControl)
                                     |
                          libfranka (built-in velocity ramp → NO reflex abort)

The joint_position_left/right JointGroupPositionController expects:
  Float64MultiArray of 7 values: [q1, q2, q3, q4, q5, q6, q7]
The joint_position_left/right JointGroupPositionController expects:
  Float64MultiArray of 7 values: [q1, q2, q3, q4, q5, q6, q7]
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

import time
import math
import threading

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint, PositionConstraint,
    OrientationConstraint, BoundingVolume, RobotState
)
from moveit_msgs.srv import GetPositionFK
from shape_msgs.msg import SolidPrimitive
from franka_custom_interfaces.action import ParallelMove


# --------------------------------------------------------------------------- #
# ARM CONFIGURATION
# --------------------------------------------------------------------------- #
ARM_CONFIG = {
    'right': {
        'group':      'franka1_manipulator',
        'joints':     ['franka1_fr3_joint1', 'franka1_fr3_joint2',
                       'franka1_fr3_joint3', 'franka1_fr3_joint4',
                       'franka1_fr3_joint5', 'franka1_fr3_joint6',
                       'franka1_fr3_joint7'],
        'jtc_topic':  '/franka1_arm_controller/follow_joint_trajectory',
        'cmd_topic':  '/cartesian_impedance_right/commands',
        'ee_link':    'franka1_fr3_hand_tcp',
    },
    'left': {
        'group':      'franka2_manipulator',
        'joints':     ['franka2_fr3_joint1', 'franka2_fr3_joint2',
                       'franka2_fr3_joint3', 'franka2_fr3_joint4',
                       'franka2_fr3_joint5', 'franka2_fr3_joint6',
                       'franka2_fr3_joint7'],
        'jtc_topic':  '/franka2_arm_controller/follow_joint_trajectory',
        'cmd_topic':  '/cartesian_impedance_left/commands',
        'ee_link':    'franka2_fr3_hand_tcp',
    },
}

STREAM_RATE_HZ   = 20.0    # Streaming rate to controller
REACH_THRESHOLD  = 0.02    # 0.02 rad tolerance to declare goal reached
STREAM_TIMEOUT_S = 30.0    # Maximum streaming time before abort
WORLD_FRAME      = 'world'


# --------------------------------------------------------------------------- #
# NODE
# --------------------------------------------------------------------------- #
class CartesianBridgeNode(Node):

    def __init__(self):
        super().__init__('cartesian_bridge_node')
        self.get_logger().info('🌉 CartesianBridgeNode starting...')

        cb = ReentrantCallbackGroup()

        # --- JTC Action Clients ---
        self._jtc_clients = {}
        for side, cfg in ARM_CONFIG.items():
            self._jtc_clients[side] = ActionClient(
                self, FollowJointTrajectory, cfg['jtc_topic'], callback_group=cb)

        # --- Current joint state cache (for FK) ---
        self._joint_states: dict[str, float] = {}
        self._js_lock = threading.Lock()
        self.create_subscription(
            JointState, '/joint_states', self._js_callback, 10,
            callback_group=cb)

        # --- MoveIt planning client ---
        self._plan_client = ActionClient(
            self, MoveGroup, 'move_action', callback_group=cb)

        # --- FK service client ---
        self._fk_client = self.create_client(
            GetPositionFK, 'compute_fk', callback_group=cb)

        # --- Impedance Command Publishers ---
        self._cmd_pubs = {}
        for side, cfg in ARM_CONFIG.items():
            self._cmd_pubs[side] = self.create_publisher(
                Float64MultiArray, cfg['cmd_topic'], 10, callback_group=cb)

        # --- ParallelMove action server ---
        self._action_server = ActionServer(
            self, ParallelMove, 'parallel_move',
            execute_callback=self._execute_parallel_move,
            callback_group=cb)

        self.get_logger().info('✅ CartesianBridgeNode ready — waiting for goals.')

    # ------------------------------------------------------------------ #
    # Joint state callback
    # ------------------------------------------------------------------ #
    def _js_callback(self, msg: JointState):
        with self._js_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_states[name] = pos

    # ------------------------------------------------------------------ #
    # Action server callback
    # ------------------------------------------------------------------ #
    def _execute_parallel_move(self, goal_handle):
        result = ParallelMove.Result()
        goal   = goal_handle.request

        side = goal.arm  # 'left' or 'right'
        if side not in ARM_CONFIG:
            self.get_logger().error(f'❌ Unknown arm "{side}"')
            goal_handle.abort()
            result.success = False
            return result

        cfg = ARM_CONFIG[side]
        self.get_logger().info(f'🎯 [{side}] ParallelMove received — planner: {goal.planner_id}')

        # ---- Step 1: Plan with MoveIt ----
        trajectory = self._plan_with_moveit(goal, cfg)
        if trajectory is None:
            self.get_logger().error(f'❌ [{side}] Planning failed via MoveIt.')
            goal_handle.abort()
            result.success = False
            return result

        self.get_logger().info(f'✅ [{side}] Plan computed. Starting Cartesian streaming...')

        # ---- Step 3: Execute via Cartesian Impedance Streaming ----
        success = self._execute_streaming(side, trajectory, goal_handle)

        result.success = success
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ------------------------------------------------------------------ #
    # MoveIt planning
    # ------------------------------------------------------------------ #
    def _plan_with_moveit(self, goal, cfg):
        """Send a planning request to MoveIt and return the joint trajectory."""
        if not self._plan_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available!')
            return None

        mg_goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name        = cfg['group']
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor     = 0.3
        req.max_acceleration_scaling_factor = 0.3

        # Explicitly set start state to current hardware state to prevent "Empty JointState" errors
        # and ensure the trajectory starts exactly where the robot currently is!
        with self._js_lock:
            current_js = dict(self._joint_states)
        
        req.start_state.is_diff = False
        req.start_state.joint_state.name = list(current_js.keys())
        req.start_state.joint_state.position = list(current_js.values())

        planner = goal.planner_id or 'PTP'
        if planner in ('PTP', 'LIN', 'CIRC'):
            req.pipeline_id = 'pilz_industrial_motion_planner'
            req.planner_id  = planner
        else:
            req.pipeline_id = 'ompl'
            req.planner_id  = 'RRTConnectkConfigDefault'

        # Build goal constraints
        constraints = Constraints()
        if goal.joint_target:
            # Joint-space goal
            for jname, jval in zip(cfg['joints'], goal.joint_target):
                jc = JointConstraint()
                jc.joint_name        = jname
                jc.position          = jval
                jc.tolerance_above   = 0.01
                jc.tolerance_below   = 0.01
                jc.weight            = 1.0
                constraints.joint_constraints.append(jc)
        else:
            # Cartesian goal (pose target)
            pose_stamped = goal.target_pose
            pc = PositionConstraint()
            pc.header         = pose_stamped.header
            pc.link_name      = cfg['ee_link']
            pc.target_point_offset.x = 0.0
            pc.target_point_offset.y = 0.0
            pc.target_point_offset.z = 0.0
            region = BoundingVolume()
            sp = SolidPrimitive()
            sp.type = SolidPrimitive.SPHERE
            sp.dimensions = [0.01]
            region.primitives.append(sp)
            region.primitive_poses.append(pose_stamped.pose)
            pc.constraint_region = region
            pc.weight = 1.0
            constraints.position_constraints.append(pc)

            oc = OrientationConstraint()
            oc.header              = pose_stamped.header
            oc.link_name           = cfg['ee_link']
            oc.orientation         = pose_stamped.pose.orientation
            oc.absolute_x_axis_tolerance = 0.05
            oc.absolute_y_axis_tolerance = 0.05
            oc.absolute_z_axis_tolerance = 0.05
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)

        req.goal_constraints.append(constraints)
        mg_goal.request  = req
        mg_goal.planning_options.plan_only = True  # PLAN ONLY — we execute manually!

        future = self._plan_client.send_goal_async(mg_goal)
        self._spin_until_done(future, timeout=15.0)
        if not future.done():
            return None
        handle = future.result()
        if not handle or not handle.accepted:
            return None

        res_future = handle.get_result_async()
        self._spin_until_done(res_future, timeout=15.0)
        if not res_future.done():
            return None

        res = res_future.result()
        if res.result.error_code.val != 1:  # 1 = SUCCESS
            self.get_logger().error(
                f'Planning failed with error code: {res.result.error_code.val}')
            return None

        traj = res.result.planned_trajectory.joint_trajectory
        if not traj.points:
            return None

        self.get_logger().info(
            f'✅ Plan computed: {len(traj.points)} joint-space points.')
        return traj

    # ------------------------------------------------------------------ #
    # Cartesian Impedance Streaming
    # ------------------------------------------------------------------ #
    def _execute_streaming(self, side, trajectory, goal_handle):
        cfg = ARM_CONFIG[side]
        pub = self._cmd_pubs[side]
        
        # 1. Convert all joint points to Cartesian Poses via MoveIt FK
        self.get_logger().info(f'🔄 [{side}] Converting {len(trajectory.points)} points to Cartesian via FK...')
        cartesian_points = []
        
        for pt in trajectory.points:
            # Call FK service
            fk_req = GetPositionFK.Request()
            fk_req.header.frame_id = WORLD_FRAME
            fk_req.fk_link_names = [cfg['ee_link']]
            
            rs = RobotState()
            rs.joint_state.name = trajectory.joint_names
            rs.joint_state.position = pt.positions
            fk_req.robot_state = rs
            
            future = self._fk_client.call_async(fk_req)
            self._spin_until_done(future, timeout=1.0)
            if not future.done():
                self.get_logger().error("FK service timeout!")
                return False
            
            res = future.result()
            if not res or not res.pose_stamped:
                self.get_logger().error("FK calculation failed!")
                return False
            
            pose = res.pose_stamped[0].pose
            # Format: [x, y, z, qw, qx, qy, qz] as expected by IDRA impedance controller
            cartesian_points.append([
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z
            ])

        # 2. Stream at 20Hz
        self.get_logger().info(f'🚀 [{side}] Starting stream to {cfg["cmd_topic"]}...')
        
        # Initial 0.5s "Hold" at the first point to let the controller settle
        first_pt = cartesian_points[0]
        hold_msg = Float64MultiArray(data=first_pt)
        for _ in range(10): # 0.5s at 20Hz
            pub.publish(hold_msg)
            time.sleep(1.0/STREAM_RATE_HZ)

        for i, pt in enumerate(cartesian_points):
            if not rclpy.ok(): return False
            
            msg = Float64MultiArray(data=pt)
            pub.publish(msg)
            
            # Optional: feedback to action client
            # feedback = ParallelMove.Feedback() ...
            
            time.sleep(1.0/STREAM_RATE_HZ)

        self.get_logger().info(f'✨ [{side}] Streaming complete!')
        return True

    # ------------------------------------------------------------------ #
    # Utility: spin until a future is done
    # ------------------------------------------------------------------ #
    def _spin_until_done(self, future, timeout=10.0):
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > timeout:
                break
            time.sleep(0.005)


# --------------------------------------------------------------------------- #
# ENTRY POINT
# --------------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = CartesianBridgeNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
