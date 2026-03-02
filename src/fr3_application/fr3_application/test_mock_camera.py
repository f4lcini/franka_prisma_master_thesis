import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import cv2
import numpy as np
from cv_bridge import CvBridge

class MockCameraNode(Node):
    def __init__(self):
        super().__init__('mock_camera_node')
        self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/camera/depth/color/points', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bridge = CvBridge()
        
        # Create a dummy image (e.g. a black box)
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        self.get_logger().info("Mock Camera started. Publishing empty Images and PointClouds to avoid Action Server crashes.")

    def timer_callback(self):
        # 1. Publish Fake Image
        msg_img = self.bridge.cv2_to_cvmsg(self.img, encoding="bgr8")
        msg_img.header.stamp = self.get_clock().now().to_msg()
        msg_img.header.frame_id = "camera_link"
        self.image_pub.publish(msg_img)

def main(args=None):
    rclpy.init(args=args)
    node = MockCameraNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
