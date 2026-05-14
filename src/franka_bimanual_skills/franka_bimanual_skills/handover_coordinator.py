#!/usr/bin/env python3
"""
================================================================================
SharedZoneManager — MUTEX + RENDEZVOUS
Role: SERVICE SERVER
Description:
    1. Mutex: /acquire_shared_zone + /release_shared_zone (for physical safety)
    2. Rendezvous: /sync_arms (for mission synchronization)
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
from std_srvs.srv import Trigger

class SharedZoneManager(Node):
    def __init__(self):
        super().__init__('shared_zone_manager')
        self._cb_group = ReentrantCallbackGroup()
        
        # --- MUTEX STATE ---
        self._zone_busy = False
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)

        # --- RENDEZVOUS STATE ---
        self._sync_arrived_count = 0
        self._sync_condition = threading.Condition(self._lock)

        self._acquire_srv = self.create_service(Trigger, '/acquire_shared_zone', self._handle_acquire, callback_group=self._cb_group)
        self._release_srv = self.create_service(Trigger, '/release_shared_zone', self._handle_release, callback_group=self._cb_group)
        self._sync_srv = self.create_service(Trigger, '/sync_arms', self._handle_sync, callback_group=self._cb_group)

        self.get_logger().info("🛡️ SharedZoneManager Ready: Mutex (/acquire, /release) + Sync (/sync_arms)")

    def _handle_acquire(self, request, response):
        self.get_logger().info("[Mutex] 📥 Access request...")
        with self._condition:
            while self._zone_busy:
                self._condition.wait()
            self._zone_busy = True
            self.get_logger().info("✅ [Mutex] ZONE ACQUIRED.")
            response.success = True
            return response

    def _handle_release(self, request, response):
        with self._condition:
            self._zone_busy = False
            self.get_logger().info("🔓 [Mutex] ZONE RELEASED.")
            self._condition.notify_all()
            response.success = True
            return response

    def _handle_sync(self, request, response):
        """Blocks until BOTH arms have called this service."""
        self.get_logger().info("[Sync] 🏳️ Arm reached synchronization point. Waiting for partner...")
        with self._sync_condition:
            self._sync_arrived_count += 1
            if self._sync_arrived_count < 2:
                self._sync_condition.wait()
            else:
                self.get_logger().info("🚀 [Sync] BOTH ARMS READY. Releasing...")
                self._sync_condition.notify_all()
                self._sync_arrived_count = 0 # Reset for next use
            
            response.success = True
            response.message = "Synchronization successful."
            return response

def main(args=None):
    rclpy.init(args=args)
    node = SharedZoneManager()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
