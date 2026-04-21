import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from franka_custom_interfaces.action import DetectObject

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import datetime
from scipy.spatial.transform import Rotation

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class ObjectLocalizationNode(Node):
    def __init__(self):
        super().__init__('object_localization_node')
        self.get_logger().info('Object Localization Node Initializing...')

        self.cv_bridge = CvBridge()
        self.latest_image = None
        self.latest_image_time = None
        self.latest_depth = None
        self.camera_intrinsics = None

        # ---- Camera Extrinsics (TF-Free) ----
        # These are overridden at launch time if needed.
        self.declare_parameter('camera_x',     0.6)
        self.declare_parameter('camera_y',    -0.6)
        self.declare_parameter('camera_z',     1.3)
        self.declare_parameter('camera_roll',  0.0)
        self.declare_parameter('camera_pitch', 0.785)   
        self.declare_parameter('camera_yaw',   1.57)    

        self._cam_pos, self._R_optical_to_world = self._build_camera_transform()

        # ---- YOLO ----
        if YOLO is None:
            self.get_logger().error('YOLO (ultralytics) is not installed.')
            self.model = None
        else:
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info('YOLOv8n loaded successfully.')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sensor_cb_group = ReentrantCallbackGroup()
        self.action_cb_group = ReentrantCallbackGroup()

        self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos, callback_group=self.sensor_cb_group)
        self.create_subscription(Image, '/camera/depth', self.depth_callback, qos, callback_group=self.sensor_cb_group)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, qos, callback_group=self.sensor_cb_group)

        self.detect_object_server = ActionServer(
            self, DetectObject, 'detect_object',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_cb_group
        )

    def _build_camera_transform(self):
        cam_pos = np.array([
            self.get_parameter('camera_x').value,
            self.get_parameter('camera_y').value,
            self.get_parameter('camera_z').value,
        ])
        roll  = self.get_parameter('camera_roll').value
        pitch = self.get_parameter('camera_pitch').value
        yaw   = self.get_parameter('camera_yaw').value
        R_body_to_world = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        R_body_to_optical = Rotation.from_euler('xz', [-np.pi / 2, -np.pi / 2]).as_matrix()
        R_opt_to_world = R_body_to_world @ R_body_to_optical.T
        return cam_pos, R_opt_to_world

    def image_callback(self, msg):
        self.latest_image = msg
        self.latest_image_time = msg.header.stamp

    def depth_callback(self, msg):
        self.latest_depth = msg

    def camera_info_callback(self, msg):
        if self.camera_intrinsics is None:
            self.camera_intrinsics = {
                'fx': msg.k[0], 'fy': msg.k[4],
                'cx': msg.k[2], 'cy': msg.k[5]
            }

    def estimate_orientation(self, depth_image, bbox):
        x1, y1, x2, y2 = bbox
        fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
        cx, cy = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']
        pts = []
        is_metric = depth_image.dtype in (np.float32, np.float64)
        for vp in range(y1, y2, 2): # Stepped for speed
            for up in range(x1, x2, 2):
                if 0 <= vp < depth_image.shape[0] and 0 <= up < depth_image.shape[1]:
                    d = float(depth_image[vp, up])
                    if d > 0.001 and not np.isnan(d):
                        z = d if is_metric else d / 1000.0
                        pts.append([(up - cx) * z / fx, (vp - cy) * z / fy, z])
        if len(pts) < 10: return [0.0, 0.0, 0.0, 1.0]
        pts = np.array(pts)
        cov = np.cov((pts - pts.mean(axis=0)).T)
        eigenvalues, vecs = np.linalg.eigh(cov)
        idx = np.argsort(eigenvalues)[::-1]
        R = vecs[:, idx].T
        if np.linalg.det(R) < 0: R[2, :] *= -1
        return Rotation.from_matrix(R).as_quat().tolist()

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        result = DetectObject.Result()
        if self.latest_image is None or self.latest_depth is None:
            goal_handle.abort()
            return result
        
        object_name = goal_handle.request.object_name
        cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, 'bgr8').copy()
        yolo_results = self.model(cv_image, verbose=False)
        
        best_box, best_conf, best_red_ratio = None, 0.0, 0.0
        
        for box in yolo_results[0].boxes:
            conf = float(box.conf[0].item())
            if object_name == 'cube' or 'cube' in object_name:
                # HSV Red Heuristic
                x1, y1, x2, y2 = [int(c) for c in box.xyxy[0].tolist()]
                roi = cv_image[max(0,y1):y2, max(0,x1):x2]
                if roi.size == 0: continue
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
                mask2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
                red_ratio = np.sum((mask1 | mask2) > 0) / roi.size
                if red_ratio > best_red_ratio and red_ratio > 0.1:
                    best_red_ratio = red_ratio
                    best_box = box
            elif conf > best_conf:
                best_box, best_conf = box, conf

        if best_box is None:
            goal_handle.abort()
            return result

        x1, y1, x2, y2 = [int(c) for c in best_box.xyxy[0].tolist()]
        u, v = (x1 + x2) // 2, (y1 + y2) // 2
        depth_image = self.cv_bridge.imgmsg_to_cv2(self.latest_depth, 'passthrough')
        depth_value = float(depth_image[v, u])
        if depth_value <= 0.0 or np.isnan(depth_value):
            depth_value = 0.5 # Fallback
            
        is_metric = depth_image.dtype in (np.float32, np.float64)
        Z_opt = depth_value if is_metric else depth_value / 1000.0
        fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
        cx, cy = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']
        P_optical = np.array([(u - cx) * Z_opt / fx, (v - cy) * Z_opt / fy, Z_opt])
        P_world = self._R_optical_to_world @ P_optical + self._cam_pos
        
        q_opt = self.estimate_orientation(depth_image, [x1, y1, x2, y2])
        q_world = (Rotation.from_matrix(self._R_optical_to_world) * Rotation.from_quat(q_opt)).as_quat()

        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = self.latest_image_time
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = P_world
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q_world
        
        result.success = True
        result.target_pose = pose
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
