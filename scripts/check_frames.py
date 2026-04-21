#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrameChecker(Node):
    def __init__(self):
        super().__init__('frame_checker')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().info("Frame Checker Node Started. Checking transformations...")

    def on_timer(self):
        frames = [
            ('world', 'table'),
            ('table', 'franka1_fr3_link0'),
            ('table', 'franka2_fr3_link0'),
            ('table', 'shared_link')
        ]
        
        print("\n--- Current Transformations ---")
        for parent, child in frames:
            try:
                t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                pos = t.transform.translation
                rot = t.transform.rotation
                print(f"[{parent} -> {child}]")
                print(f"  Pos: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")
                print(f"  Ori: x={rot.x:.3f}, y={rot.y:.3f}, z={rot.z:.3f}, w={rot.w:.3f}")
            except TransformException as e:
                print(f"Could not transform {parent} to {child}: {e}")

def main():
    rclpy.init()
    node = FrameChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
