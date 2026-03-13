"""
================================================================================
Author: Falco Robotics 
Code Description:
[ROLE]: ACTION SERVER (Provides: /detect_object)

This ROS 2 node performs 3D object localization by fusing 2D visual object 
detection (using YOLOv8) with 3D depth data (from an aligned depth image). 
It exposes a ROS 2 Action Server that listens for a target object name from a 
client (e.g., the Behavior Tree), locates it in the camera's RGB stream, 
deprojects the 2D bounding box center to a 3D point using the depth image and 
camera intrinsics, estimates the object's orientation via PCA on the local depth 
region, and returns the full 6DoF pose.

A debug image is saved to /mm_ws/yolo_detection_result.jpg on each detection 
for manual verification of position and orientation accuracy.

Pipeline: Perception -> Action Server
================================================================================
"""

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
from scipy.spatial.transform import Rotation

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class ObjectLocalizationNode(Node):
    def __init__(self):
        super().__init__('object_localization_node')
        self.get_logger().info('Object Localization Node Initializing...')
        
        # ---- Core ----
        self.cv_bridge = CvBridge()
        self.latest_image = None
        self.latest_image_time = None
        self.latest_depth = None
        self.camera_intrinsics = None

        # ---- YOLO ----
        if YOLO is None:
            self.get_logger().error('YOLO (ultralytics) is not installed.')
            self.model = None
        else:
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info('YOLOv8n loaded successfully.')
        
        # ---- QoS (Best Effort to match RealSense) ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ---- Callback Groups ----
        self.sensor_cb_group = ReentrantCallbackGroup()
        self.action_cb_group = ReentrantCallbackGroup()
        
        # ---- Subscribers ----
        self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback, qos,
            callback_group=self.sensor_cb_group
        )
        self.create_subscription(
            Image, '/camera/depth',
            self.depth_callback, qos,
            callback_group=self.sensor_cb_group
        )
        self.create_subscription(
            CameraInfo, '/camera/camera_info',
            self.camera_info_callback, qos,
            callback_group=self.sensor_cb_group
        )
        
        # ---- Action Server ----
        self.detect_object_server = ActionServer(
            self, DetectObject, 'detect_object',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_cb_group
        )
        
        self.get_logger().info('Node ready. Waiting for sensor data...')

    # ===================== SENSOR CALLBACKS =====================

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
            self.get_logger().info(
                f'Intrinsics: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}'
            )

    # ===================== PCA ORIENTATION =====================

    def estimate_orientation(self, depth_image, bbox):
        """PCA orientation estimation on depth region. Returns quaternion [x, y, z, w]."""
        x1, y1, x2, y2 = bbox
        fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
        cx, cy = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']
        
        pts = []
        for vp in range(y1, y2):
            for up in range(x1, x2):
                if 0 <= vp < depth_image.shape[0] and 0 <= up < depth_image.shape[1]:
                    d = float(depth_image[vp, up])
                    if d > 0.0 and not np.isnan(d):
                        z = d / 1000.0
                        pts.append([(up - cx) * z / fx, (vp - cy) * z / fy, z])
        
        if len(pts) < 10:
            self.get_logger().warn(f'PCA: only {len(pts)} valid points, using identity quaternion.')
            return [0.0, 0.0, 0.0, 1.0]
        
        pts = np.array(pts)
        cov = np.cov((pts - pts.mean(axis=0)).T)
        eigenvalues, vecs = np.linalg.eigh(cov)
        
        idx = np.argsort(eigenvalues)[::-1]
        eigenvalues = eigenvalues[idx]
        vecs = vecs[:, idx]
        
        R = vecs.T
        if np.linalg.det(R) < 0:
            R[2, :] *= -1
        
        quat = Rotation.from_matrix(R).as_quat()  # [x, y, z, w]
        
        self.get_logger().info(
            f'PCA: eigenvalues=[{eigenvalues[0]:.6f}, {eigenvalues[1]:.6f}, {eigenvalues[2]:.6f}], '
            f'quat=[{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]'
        )
        return quat.tolist()

    # ===================== ACTION SERVER =====================

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Goal received: "{goal_request.object_name}"')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Goal cancelled by client.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing detection goal...')
        result = DetectObject.Result()
        
        # --- Prerequisites ---
        if self.model is None:
            self.get_logger().error('YOLO not available.')
            result.success = False
            goal_handle.abort()
            return result
        
        if self.latest_image is None or self.latest_depth is None or self.camera_intrinsics is None:
            self.get_logger().error(
                f'Missing data — Image:{self.latest_image is not None} '
                f'Depth:{self.latest_depth is not None} '
                f'Intrinsics:{self.camera_intrinsics is not None}'
            )
            result.success = False
            goal_handle.abort()
            return result
        
        object_name = goal_handle.request.object_name
        self.get_logger().info(f'Searching for: "{object_name}"')
        
        # --- Semantic Label Mapping ---
        # Maps user/task-layer names to standard YOLOv8 COCO classes
        LABEL_MAP = {
            "green_ball": ["sports ball", "apple", "orange"],
            "green ball": ["sports ball", "apple", "orange"],
            "open_box": ["suitcase", "bowl", "book", "box"],
            "open box": ["suitcase", "bowl", "book", "box"],
            "table": ["dining table", "desk"]
        }
        
        # Determine which YOLO labels we should look for
        search_targets = [object_name]
        if object_name.lower() in LABEL_MAP:
            search_targets.extend(LABEL_MAP[object_name.lower()])
        
        # --- YOLO Inference ---
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, 'bgr8').copy()
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            result.success = False
            goal_handle.abort()
            return result
        
        results = self.model(cv_image, verbose=False)
        
        # Parse all detections
        best_box, best_conf = None, 0.0
        all_labels = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0].item())
            label = self.model.names[cls_id]
            conf = float(box.conf[0].item())
            all_labels.append(f'{label}({conf:.0%})')
            
            # Match if the label is exactly the target or in the semantic mapping
            if (label == object_name or label in search_targets) and conf > best_conf:
                best_box, best_conf = box, conf
        
        self.get_logger().info(f'All detections: {", ".join(all_labels) if all_labels else "NONE"}')
        
        if best_box is None:
            self.get_logger().error(f'"{object_name}" (mapped to {search_targets}) not found in frame.')
            result.success = False
            goal_handle.abort()
            return result
        
        # --- 2D Center ---
        x1, y1, x2, y2 = [int(c) for c in best_box.xyxy[0].tolist()]
        u = (x1 + x2) // 2
        v = (y1 + y2) // 2
        self.get_logger().info(f'Found "{object_name}" at pixel ({u}, {v}), conf={best_conf:.2f}')
        
        # --- Depth Lookup ---
        try:
            depth_image = self.cv_bridge.imgmsg_to_cv2(self.latest_depth, 'passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'Depth CvBridge error: {e}')
            result.success = False
            goal_handle.abort()
            return result
        
        depth_value = float(depth_image[v, u])
        
        # DIAGNOSTIC: understand depth image format
        self.get_logger().info(
            f'DEPTH DIAGNOSTIC — dtype:{depth_image.dtype}, encoding:"{self.latest_depth.encoding}", '
            f'raw_center_value:{depth_value}, '
            f'image_min:{np.nanmin(depth_image)}, image_max:{np.nanmax(depth_image)}, '
            f'shape:{depth_image.shape}'
        )
        
        if depth_value <= 0.0 or np.isnan(depth_value):
            self.get_logger().warn('Center depth invalid, searching neighborhood...')
            depth_value = self._search_valid_depth(depth_image, u, v, radius=15)
            if depth_value is None:
                self.get_logger().error('No valid depth in neighborhood.')
                result.success = False
                result.message = 'Depth is zero/NaN at object location.'
                goal_handle.abort()
                return result
        
        # --- 3D Deprojection ---
        Z = depth_value / 1000.0
        fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
        cx, cy = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        self.get_logger().info(f'3D Position: X={X:.3f} Y={Y:.3f} Z={Z:.3f}m (depth={depth_value:.0f}mm)')
        
        # --- Orientation (PCA) ---
        quat = self.estimate_orientation(depth_image, [x1, y1, x2, y2])
        
        # --- Debug Image (saved to disk for manual inspection) ---
        self._save_debug_image(cv_image, [x1, y1, x2, y2], (X, Y, Z), quat, best_conf, object_name)
        
        # --- Result ---
        pose = PoseStamped()
        pose.header.frame_id = self.latest_depth.header.frame_id
        pose.header.stamp = self.latest_image_time
        pose.pose.position.x = X
        pose.pose.position.y = Y
        pose.pose.position.z = Z
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        result.success = True
        result.target_pose = pose
        result.message = (
            f"Localized '{object_name}': Pos({X:.3f},{Y:.3f},{Z:.3f}) "
            f"Quat({quat[0]:.3f},{quat[1]:.3f},{quat[2]:.3f},{quat[3]:.3f})"
        )
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result

    # ===================== HELPERS =====================

    def _search_valid_depth(self, depth_image, u, v, radius=15):
        """Search expanding rings around (u,v) for a valid depth value."""
        for r in range(1, radius + 1):
            for i in range(-r, r + 1):
                for j in range(-r, r + 1):
                    nv, nu = v + i, u + j
                    if 0 <= nv < depth_image.shape[0] and 0 <= nu < depth_image.shape[1]:
                        d = float(depth_image[nv, nu])
                        if d > 0.0 and not np.isnan(d):
                            return d
        return None

    def _save_debug_image(self, cv_image, bbox, pos_3d, quat, conf, name):
        """Save an annotated debug image with bbox, position, and PCA axes to disk."""
        img = cv_image.copy()
        x1, y1, x2, y2 = bbox
        X, Y, Z = pos_3d
        fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
        cx, cy = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']
        cu, cv_pt = (x1 + x2) // 2, (y1 + y2) // 2
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # Bounding box + center
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(img, (cu, cv_pt), 6, (0, 0, 255), -1)
        
        # Text overlays
        cv2.putText(img, f'{name} ({conf:.0%})', (x1, y1 - 10), font, 0.5, (0, 255, 0), 1)
        cv2.putText(img, f'X:{X:.3f} Y:{Y:.3f} Z:{Z:.3f}m', (x1, y2 + 20), font, 0.5, (255, 255, 255), 1)
        cv2.putText(img, f'q:[{quat[0]:.3f},{quat[1]:.3f},{quat[2]:.3f},{quat[3]:.3f}]',
                    (x1, y2 + 40), font, 0.45, (200, 200, 200), 1)
        
        # PCA orientation axes (R=X, G=Y, B=Z)
        R = Rotation.from_quat(quat).as_matrix()
        axis_len = max(0.05, Z * 0.05)
        colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]
        labels = ['X', 'Y', 'Z']
        for i in range(3):
            end = np.array([X, Y, Z]) + R[:, i] * axis_len
            if end[2] > 0:
                eu = int(fx * end[0] / end[2] + cx)
                ev = int(fy * end[1] / end[2] + cy)
                cv2.arrowedLine(img, (cu, cv_pt), (eu, ev), colors[i], 2, tipLength=0.2)
                cv2.putText(img, labels[i], (eu + 5, ev + 5), font, 0.4, colors[i], 1)
        
        path = '/mm_ws/yolo_detection_resulted.jpg'
        cv2.imwrite(path, img)
        self.get_logger().info(f'Debug image saved: {path}')


# ===================== MAIN =====================

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
