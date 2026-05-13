import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from franka_custom_interfaces.action import DetectObject

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import datetime
import os
import time
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

        self._cam_pos, self._R_optical_to_table = self._build_camera_transform()
        self.get_logger().info(f"📸 CAMERA POS: {self._cam_pos}")
        self.get_logger().info(f"📸 CAMERA ROT MATRIX:\n{self._R_optical_to_table}")

        # ---- YOLO26 ----
        if YOLO is None:
            self.get_logger().error('YOLO (ultralytics) is not installed.')
            self.model = None
        else:
            # Carichiamo il modello ONNX esportato per massime prestazioni su CPU
            model_path = '/mm_ws/yolo26s.onnx'
            if os.path.exists(model_path):
                self.model = YOLO(model_path, task='detect')
                self.get_logger().info(f'YOLO26s (ONNX) caricato correttamente da {model_path}')
            else:
                # Fallback al modello Medium di YOLOv8 (ultralytics lo scaricherà in automatico se manca)
                self.model = YOLO('yolov8m.pt')
                self.get_logger().info('YOLOv8 Medium caricato per maggiore precisione!')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sensor_cb_group = ReentrantCallbackGroup()
        self.action_cb_group = ReentrantCallbackGroup()

        # Debug Image and Marker Publishers
        self.debug_image_pub = self.create_publisher(Image, '/yolo_debug_image', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/yolo_detected_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/yolo_markers', 10)

        # Declare Topic Parameters
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        self.get_logger().info(f'Subscribing to Image: {image_topic}')
        self.get_logger().info(f'Subscribing to Depth: {depth_topic}')

        self.create_subscription(Image, image_topic, self.image_callback, qos, callback_group=self.sensor_cb_group)
        self.create_subscription(Image, depth_topic, self.depth_callback, qos, callback_group=self.sensor_cb_group)
        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, qos, callback_group=self.sensor_cb_group)

        self.detect_object_server = ActionServer(
            self, DetectObject, 'detect_object',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_cb_group
        )
        self._frame_counter = 0

    def _build_camera_transform(self):
        cam_pos = np.array([
            self.get_parameter('camera_x').value,
            self.get_parameter('camera_y').value,
            self.get_parameter('camera_z').value,
        ])
        roll  = self.get_parameter('camera_roll').value
        pitch = self.get_parameter('camera_pitch').value
        yaw   = self.get_parameter('camera_yaw').value
        # I valori da robot_poses.yaml (roll, pitch, yaw) ora appartengono 
        # DIRETTAMENTE al frame ottico a colori! Nessuna rotazione aggiuntiva richiesta.
        R_opt_to_table = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        return cam_pos, R_opt_to_table

    def image_callback(self, msg):
        self.latest_image = msg
        self.latest_image_time = msg.header.stamp
        
        # Latency Optimization: Skip frames to avoid backlog
        self._frame_counter += 1
        if self._frame_counter % 9 != 0: # Process at ~3.3Hz instead of 30Hz
            return

        # Continuous Live Detection for Debugging
        if self.model is not None and self.camera_intrinsics is not None and self.latest_depth is not None:
            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
                depth_image = self.cv_bridge.imgmsg_to_cv2(self.latest_depth, 'passthrough')
                results = self.model(cv_image, verbose=False, conf=0.3)
                
                debug_img = cv_image.copy()
                marker_array = MarkerArray()
                
                for i, box in enumerate(results[0].boxes):
                    x1, y1, x2, y2 = [int(c) for c in box.xyxy[0].tolist()]
                    cls_id = int(box.cls[0].item())
                    cls_name = self.model.names[cls_id]
                    conf = float(box.conf[0].item())
                    
                    # Draw 2D
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(debug_img, f"{cls_name} {conf:.2f}", (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Calculate 3D Position (simple centroid depth)
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2
                    if 0 <= v < depth_image.shape[0] and 0 <= u < depth_image.shape[1]:
                        d = float(depth_image[v, u])
                        if d > 0.001 and not np.isnan(d):
                            z_opt = d if depth_image.dtype in (np.float32, np.float64) else d / 1000.0
                            fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
                            cx, cy = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']
                            p_opt = np.array([(u - cx) * z_opt / fx, (v - cy) * z_opt / fy, z_opt])
                            p_table = self._R_optical_to_table @ p_opt + self._cam_pos
                            
                            # Publish PoseStamped
                            ps = PoseStamped()
                            ps.header.frame_id = 'table'
                            ps.header.stamp = msg.header.stamp
                            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p_table
                            self.pose_pub.publish(ps)
                            
                            # Add Marker for RViz
                            marker = Marker()
                            marker.header.frame_id = 'table'
                            marker.header.stamp = msg.header.stamp
                            marker.ns = "yolo_detections"
                            marker.id = i
                            marker.type = Marker.SPHERE
                            marker.action = Marker.ADD
                            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = p_table
                            marker.scale.x = marker.scale.y = marker.scale.z = 0.05
                            marker.color.a = 1.0
                            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0
                            marker_array.markers.append(marker)
                
                self.debug_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(debug_img, 'bgr8'))
                self.marker_pub.publish(marker_array)
            except Exception as e:
                self.get_logger().warn(f"Error in live detection: {e}")

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

    def execute_callback(self, goal_handle):
        import time
        result = DetectObject.Result()
        object_name = goal_handle.request.object_name
        self._cam_pos, self._R_optical_to_table = self._build_camera_transform()
        
        # --- CONFIGURAZIONE FILTRO ---
        num_samples = 1 # Cambia a 1 per disattivare il filtro e avere risposta immediata
        # -----------------------------

        samples_x, samples_y = [], []
        self.get_logger().info(f"🔍 Localizzazione di {object_name} ({num_samples} campioni)...")

        for i in range(num_samples):
            if self.latest_image is None:
                time.sleep(0.1)
                continue
            
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, 'bgr8').copy()
            yolo_results = self.model(cv_image, verbose=False)
            
            best_box, best_conf = None, 0.0
            for box in yolo_results[0].boxes:
                conf = float(box.conf[0].item())
                if object_name.lower() in self.model.names[int(box.cls[0].item())].lower() and conf > best_conf:
                    best_box, best_conf = box, conf

            if best_box is not None:
                x1, y1, x2, y2 = [int(c) for c in best_box.xyxy[0].tolist()]
                u, v = (x1 + x2) // 2, y2 # Base dell'oggetto
                
                fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
                cx, cy = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']
                
                v_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
                v_table = self._R_optical_to_table @ v_opt
                lam = -self._cam_pos[2] / v_table[2]
                P_table = self._cam_pos + lam * v_table
                
                samples_x.append(P_table[0])
                samples_y.append(P_table[1])
            
            if num_samples > 1: time.sleep(0.05)

        if not samples_x:
            self.get_logger().error(f"❌ {object_name} non trovato.")
            goal_handle.abort()
            return result

        final_x = np.median(samples_x)
        final_y = np.median(samples_y)
        
        # --- RIMOZIONE INVERSIONE ASSI ---
        # La matrice è ora calibrata in modo puramente ottico e senza rotazioni fittizie del Tag.
        # final_x = final_x
        # final_y = final_y
        # ----------------------------------------------------------------------

        self.get_logger().info(f"📍 RISULTATO FINALE: x={final_x:.3f}, y={final_y:.3f}")
        
        pose = PoseStamped()
        pose.header.frame_id = object_name  # Passiamo il nome dell'oggetto qui!
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(final_x), float(final_y), 0.0
        pose.pose.orientation.w = 1.0
        
        result.success = True
        result.target_pose = pose
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizationNode()
    executor = MultiThreadedExecutor(num_threads=10)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
