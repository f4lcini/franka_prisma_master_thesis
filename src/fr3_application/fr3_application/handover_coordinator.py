#!/usr/bin/env python3
"""
================================================================================
HandoverCoordinator Node — DUAL SERVICE RENDEZVOUS
Role: SERVICE SERVER — provides /donor_ready and /recipient_ready
Description:
    Implements a Rendezvous pattern for bimanual handover synchronization.
    TWO separate services avoid the ROS 2 single-service-one-request-at-a-time
    limitation. Each arm calls its own service and blocks until the partner
    has called their service, then both are released simultaneously.
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import threading

from franka_custom_interfaces.srv import HandoverReady


class HandoverCoordinator(Node):
    def __init__(self):
        super().__init__('handover_coordinator')
        self._cb_group = ReentrantCallbackGroup()

        self._lock = threading.Lock()
        self._donor_arrived = threading.Event()
        self._recipient_arrived = threading.Event()

        # Two SEPARATE services — one per arm — to avoid single-service deadlock
        self._donor_srv = self.create_service(
            HandoverReady,
            '/donor_ready',
            self._handle_donor,
            callback_group=self._cb_group
        )
        self._recipient_srv = self.create_service(
            HandoverReady,
            '/recipient_ready',
            self._handle_recipient,
            callback_group=self._cb_group
        )

        self.get_logger().info("✅ HandoverCoordinator ready: /donor_ready + /recipient_ready")

    def _reset(self):
        """Reset both events for the next handover cycle."""
        self._donor_arrived.clear()
        self._recipient_arrived.clear()

    def _handle_donor(self, request, response):
        """Donor calls this. Blocks until recipient also arrives."""
        timeout = float(request.timeout_sec) if request.timeout_sec > 0 else 120.0
        self.get_logger().info(f"[HandoverCoordinator] 🎁 Donor registered. Waiting for recipient...")

        self._donor_arrived.set()
        ok = self._recipient_arrived.wait(timeout=timeout)

        if not ok:
            self.get_logger().error("[HandoverCoordinator] ⏱ Timeout: recipient never arrived!")
            self._reset()
            response.success = False
            response.message = "Timeout: recipient never arrived."
            return response

        self.get_logger().info("[HandoverCoordinator] ✅ Rendezvous achieved! Releasing donor.")
        response.success = True
        response.message = "Rendezvous successful."

        # Reset after both have been served (last one resets)
        with self._lock:
            if self._donor_arrived.is_set() and self._recipient_arrived.is_set():
                self._reset()
        return response

    def _handle_recipient(self, request, response):
        """Recipient calls this. Blocks until donor also arrives."""
        timeout = float(request.timeout_sec) if request.timeout_sec > 0 else 120.0
        self.get_logger().info(f"[HandoverCoordinator] 🤝 Recipient registered. Waiting for donor...")

        self._recipient_arrived.set()
        ok = self._donor_arrived.wait(timeout=timeout)

        if not ok:
            self.get_logger().error("[HandoverCoordinator] ⏱ Timeout: donor never arrived!")
            self._reset()
            response.success = False
            response.message = "Timeout: donor never arrived."
            return response

        self.get_logger().info("[HandoverCoordinator] ✅ Rendezvous achieved! Releasing recipient.")
        response.success = True
        response.message = "Rendezvous successful."

        with self._lock:
            if self._donor_arrived.is_set() and self._recipient_arrived.is_set():
                self._reset()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HandoverCoordinator()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
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
