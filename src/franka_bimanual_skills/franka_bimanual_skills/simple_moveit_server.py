#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import traceback

from .robot_control_api import RobotControlAPI
from .skill_behaviors import SkillBehaviors
from .config import DEFAULT_OFFSETS

class SimpleMoveItServer(Node):
    def __init__(self):
        super().__init__('simple_moveit_server')
        
        # CRITICAL FIX: To avoid deadlock in ROS 2 Humble when an action calls another action,
        # we separate the Server and Client into different Callback Groups.
        self._server_cb_group = ReentrantCallbackGroup()
        self._client_cb_group = ReentrantCallbackGroup()
        
        self.get_logger().info("==============================================")
        self.get_logger().info(" Python Action Servers Starting (Strategic CBG)... ")
        self.get_logger().info("==============================================")

        # Initialize Robot Control API for mechanical API
        self.robot_control_api = RobotControlAPI(self, self._client_cb_group)
        
        # Initialize Skill Behaviors linking mechanical components to semantics
        self.skill_behaviors = SkillBehaviors(self, self.robot_control_api, self._server_cb_group)

        # --- PICK & PLACE PARAMETERS ---
        for param_name, default_val in DEFAULT_OFFSETS.items():
            self.declare_parameter(param_name, default_val)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMoveItServer()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=12)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down gracefully...")
    except Exception as e:
        node.get_logger().fatal(f"Unhandled Execution Exception: {e}")
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
