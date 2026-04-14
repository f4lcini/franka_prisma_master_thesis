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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

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

        # ---- Camera Extrinsics (TF-Free world-frame output) -------------------------
        # These parameters MUST match the camera <pose> in bimanual_custom.world.
        # They are declared as ROS parameters so they can be overridden at launch time
        # without modifying this file (e.g. ros2 run ... --ros-args -p camera_x:=0.7).
        #
        # SDF pose: <pose>0.6 1.0 1.0  roll=0  pitch=0.785  yaw=-1.57</pose>
        # The SDF RPY uses extrinsic XYZ convention (roll around world X, then pitch
        # around world Y, then yaw around world Z).
        #
        # Gazebo IGN RGBD sensor: the sensor frame (camera/link/rgb_camera) is
        # rotated from the body link by Rx(-pi/2) Rz(-pi/2), converting the body
        # convention (X=right, Y=up, Z=back) to ROS optical
        # (X=right, Y=down, Z=into scene).
        #
        # R_optical_to_world = R_body_to_world @ R_body_to_optical.T
        # P_world = R_optical_to_world @ P_optical + cam_pos
        # -------------------------------------------------------------------------------
        self.declare_parameter('camera_x',     0.6)
        self.declare_parameter('camera_y',     1.0)
        self.declare_parameter('camera_z',     1.0)
        self.declare_parameter('camera_roll',  0.0)
        self.declare_parameter('camera_pitch', 0.785)   # pi/4 -> 45 deg tilt down
        self.declare_parameter('camera_yaw',  -1.57)    # -pi/2 -> rotated 90 deg CW
        
        # ---- Lab Scaling & Topics ----
        self.declare_parameter('image_topic',       '/camera/image_raw')
        self.declare_parameter('depth_topic',       '/camera/depth')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('use_sensor_data_qos', False)

        self._cam_pos, self._R_optical_to_world = self._build_camera_transform()
        self.get_logger().info(
            f'Camera TF-Free: pos={self._cam_pos.tolist()}, '
            f'R_opt_to_world=\n{np.round(self._R_optical_to_world, 4)}'
        )

        # ---- YOLO ----
        if YOLO is None:
            self.get_logger().error('YOLO (ultralytics) is not installed.')
            self.model = None
        else:
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info('YOLOv8n loaded successfully.')

        # ---- QoS Strategy ----
        if self.get_parameter('use_sensor_data_qos').value:
            self.get_logger().info("Using SensorDataQoS (Best Effort) for lab hardware.")
            qos = qos_profile_sensor_data
        else:
            self.get_logger().info("Using Reliable QoS for simulation.")
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

        # ---- Callback Groups ----
        self.sensor_cb_group = ReentrantCallbackGroup()
        self.action_cb_group = ReentrantCallbackGroup()

        # ---- Subscribers ----
        self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self.image_callback, qos,
            callback_group=self.sensor_cb_group
        )
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, qos,
            callback_group=self.sensor_cb_group
        )
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
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

    # ===================== CAMERA TRANSFORM (TF-FREE) =====================

    def _build_camera_transform(self):
        """Build the optical-frame -> world rotation matrix from SDF parameters.

        Returns
        -------
        cam_pos : np.ndarray shape (3,)
            Camera position in world frame.
        R_opt_to_world : np.ndarray shape (3,3)
            Rotation matrix: P_world_rel = R_opt_to_world @ P_optical
        """
        cam_pos = np.array([
            self.get_parameter('camera_x').value,
            self.get_parameter('camera_y').value,
            self.get_parameter('camera_z').value,
        ])
        roll  = self.get_parameter('camera_roll').value
        pitch = self.get_parameter('camera_pitch').value
        yaw   = self.get_parameter('camera_yaw').value

        # Body -> world  (extrinsic RPY: Rx(roll) * Ry(pitch) * Rz(yaw) applied in world axes)
        R_body_to_world = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

        # Body -> optical  (intrinsic xz: Rx(-pi/2) then Rz(-pi/2))
        # Equivalently in extrinsic form: Rz(-pi/2) @ Rx(-pi/2)
        R_body_to_optical = Rotation.from_euler('xz', [-np.pi / 2, -np.pi / 2]).as_matrix()

        # optical -> world
        R_opt_to_world = R_body_to_world @ R_body_to_optical.T
        return cam_pos, R_opt_to_world

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
        is_metric = depth_image.dtype in (np.float32, np.float64)
        for vp in range(y1, y2):
            for up in range(x1, x2):
                if 0 <= vp < depth_image.shape[0] and 0 <= up < depth_image.shape[1]:
                    d = float(depth_image[vp, up])
                    if d > 0.001 and not np.isnan(d):
                        z = d if is_metric else d / 1000.0
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
        LABEL_MAP = {
            "green_ball": ["sports ball", "apple", "orange"],
            "green ball": ["sports ball", "apple", "orange"],
            "open_box": ["suitcase", "bowl", "book", "box"],
            "open box": ["suitcase", "bowl", "book", "box"],
            "red_cube": ["traffic light", "stop sign", "suitcase", "book", "apple", "refrigerator"],
            "red cube": ["traffic light", "stop sign", "suitcase", "book", "apple", "refrigerator"],
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
        
        # Detect encoding: Gazebo IGN outputs 32FC1 (float32, in meters).
        # Real RealSense cameras output 16UC1 (uint16, in mm).
        is_metric = self.latest_depth.encoding in ('32FC1', '64FC1') or depth_image.dtype in (np.float32, np.float64)
        
        # Validity threshold adapts to encoding
        invalid_threshold = 0.001 if is_metric else 1.0  # <1mm in meters, or <1 raw unit
        
        if depth_value <= invalid_threshold or np.isnan(depth_value):
            self.get_logger().warn(f'Center depth invalid ({depth_value}, is_metric={is_metric}), searching neighborhood...')
            depth_value = self._search_valid_depth(depth_image, u, v, radius=15)
            if depth_value is None:
                self.get_logger().error('No valid depth in neighborhood.')
                result.success = False
                result.message = 'Depth is zero/NaN at object location.'
                goal_handle.abort()
                return result
        
        # --- 3D Deprojection (optical frame) ---
        Z_opt = depth_value if is_metric else depth_value / 1000.0
        fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
        cx, cy = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']
        X_opt = (u - cx) * Z_opt / fx
        Y_opt = (v - cy) * Z_opt / fy
        self.get_logger().info(
            f'[CAM-FRAME]   X={X_opt:.3f}  Y={Y_opt:.3f}  Z={Z_opt:.3f} m'
        )

        # --- TF-Free transform to world frame ----------------------------------
        # P_world = R_optical_to_world @ P_optical + cam_position
        # No TF tree lookup needed: uses hardcoded (but ROS-parameter-overridable)
        # camera extrinsics built from bimanual_custom.world SDF pose.
        P_optical = np.array([X_opt, Y_opt, Z_opt])
        P_world   = self._R_optical_to_world @ P_optical + self._cam_pos
        X_w, Y_w, Z_w = P_world
        self.get_logger().info(
            f'[WORLD-FRAME] X={X_w:.3f}  Y={Y_w:.3f}  Z={Z_w:.3f} m'
        )
        # -----------------------------------------------------------------------

        # --- Orientation (PCA, rotated to world frame) ---
        q_opt = self.estimate_orientation(depth_image, [x1, y1, x2, y2])  # [x,y,z,w]
        q_world = (
            Rotation.from_matrix(self._R_optical_to_world)
            * Rotation.from_quat(q_opt)
        ).as_quat()  # [x,y,z,w]

        # --- Debug Image ---
        self._save_debug_image(cv_image, [x1, y1, x2, y2], (X_w, Y_w, Z_w), q_world.tolist(), best_conf, object_name)

        # --- Result (always in 'world' frame — no downstream TF lookup needed) ---
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = self.latest_image_time
        pose.pose.position.x = X_w
        pose.pose.position.y = Y_w
        pose.pose.position.z = Z_w
        pose.pose.orientation.x = float(q_world[0])
        pose.pose.orientation.y = float(q_world[1])
        pose.pose.orientation.z = float(q_world[2])
        pose.pose.orientation.w = float(q_world[3])
        
        result.success = True
        result.target_pose = pose
        result.message = (
            f"Localized '{object_name}' [world]: "
            f"Pos({X_w:.3f},{Y_w:.3f},{Z_w:.3f}) "
            f"Quat({q_world[0]:.3f},{q_world[1]:.3f},{q_world[2]:.3f},{q_world[3]:.3f})"
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
