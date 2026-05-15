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
import asyncio
from scipy.spatial.transform import Rotation

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

from .config import TARGET_OFFSETS


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

        # ---- YOLOv26 Configuration ----
        self.num_samples = 1 # <--- CAMBIA QUESTO NUMERO PER VELOCIZZARE (es. 1)
        
        if YOLO is None:
            self.get_logger().error('Libreria YOLO (ultralytics) non trovata.')
            self.model = None
        else:
            # Questo comando scarica automaticamente 'yolo26m.pt' al primo avvio
            self.model = YOLO("yolo26m.pt")
            self.get_logger().info('✅ YOLOv26m inizializzato (download automatico se necessario).')

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

        # --- DUAL ACTION SERVERS (One per Arm) ---
        self.detect_left_server = ActionServer(
            self, DetectObject, 'detect_object_left',
            execute_callback=self.execute_callback_left,
            callback_group=self.action_cb_group
        )
        self.detect_right_server = ActionServer(
            self, DetectObject, 'detect_object_right',
            execute_callback=self.execute_callback_right,
            callback_group=self.action_cb_group
        )
        
        # Timer per il debug live
        self.create_timer(0.1, self.debug_timer_callback, callback_group=self.sensor_cb_group)
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
        """Salva l'ultimo frame ricevuto senza processarlo (Latenza Zero)."""
        self.latest_image = msg
        self.latest_image_time = msg.header.stamp

    def debug_timer_callback(self):
        """Esegue l'inferenza di debug in modo asincrono."""
        if self.latest_image is None or self.model is None:
            return

        try:
            # Convertiamo l'ultimo frame disponibile
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
            
            # OTTIMIZZAZIONE: Inferenza a bassa risoluzione (320px) per il debug live
            results = self.model(cv_image, verbose=False, conf=0.3, imgsz=320)
            
            # Usiamo il metodo nativo .plot() di Ultralytics che è estremamente ottimizzato
            annotated_frame = results[0].plot(labels=True, boxes=True)
            
            # Pubblichiamo l'immagine di debug
            debug_msg = self.cv_bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
            debug_msg.header = self.latest_image.header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Errore nel timer di debug: {e}")

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

    def execute_callback_left(self, goal_handle):
        return self._execute_common(goal_handle, side="left")

    def execute_callback_right(self, goal_handle):
        return self._execute_common(goal_handle, side="right")

    def _execute_common(self, goal_handle, side="left"):
        """Logica di localizzazione condivisa (Sincrona con MultiThread)."""
        result = DetectObject.Result()
        object_name = goal_handle.request.object_name
        self._cam_pos, self._R_optical_to_table = self._build_camera_transform()
        
        self.get_logger().info(f"🔍 [{side.upper()}] Cerco '{object_name}' nel mio spazio di lavoro...")

        # Raccogliamo campioni per stabilità
        samples = []
        for _ in range(self.num_samples):
            if self.latest_image is None or self.camera_intrinsics is None:
                time.sleep(0.1)
                continue
            
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
            # Usiamo risoluzione standard per la precisione di pick
            yolo_results = self.model(cv_image, verbose=False, imgsz=640)
            
            # Troviamo tutti i candidati validi
            candidates = []
            for box in yolo_results[0].boxes:
                conf = float(box.conf[0].item())
                if object_name.lower() in self.model.names[int(box.cls[0].item())].lower() and conf > 0.25:
                    # Calcoliamo la posizione 3D del candidato
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    u, v = (x1 + x2) / 2.0, y2
                    v_opt = np.array([(u - self.camera_intrinsics['cx']) / self.camera_intrinsics['fx'], 
                                     (v - self.camera_intrinsics['cy']) / self.camera_intrinsics['fy'], 1.0])
                    v_table = self._R_optical_to_table @ v_opt
                    lam = -self._cam_pos[2] / v_table[2]
                    p_table = self._cam_pos + lam * v_table
                    candidates.append((p_table, conf))

            # Filtro spaziale con sovrapposizione per zona condivisa (handover)
            if side == "left":
                # Il sinistro vede fino a +5cm nel lato destro
                valid = [c for c in candidates if c[0][0] < 0.05]
            else:
                # Il destro vede fino a -5cm nel lato sinistro
                valid = [c for c in candidates if c[0][0] >= -0.05]

            if candidates:
                self.get_logger().info(f"🔍 [{side.upper()}] {len(candidates)} oggetti rilevati. Validi per questo lato: {len(valid)}")
                for idx, c in enumerate(candidates):
                    status = "VALIDO" if (side == "left" and c[0][0] < 0.05) or (side == "right" and c[0][0] >= -0.05) else "FUORI_ZONA"
                    self.get_logger().info(f"   -> [{status}] X={c[0][0]:.3f}, Conf={c[1]:.2f}")

            if valid:
                # Prendiamo il migliore nel proprio lato
                best = max(valid, key=lambda x: x[1])
                samples.append(best[0])
            
            time.sleep(0.05)

        if not samples:
            self.get_logger().error(f"❌ [{side.upper()}] '{object_name}' non trovato nel mio lato.")
            goal_handle.abort()
            return result

        # Mediana finale per robustezza
        final_pos = np.median(np.array(samples), axis=0)
        
        # --- APPLICAZIONE OFFSET Y DINAMICO DA CONFIG.PY ---
        obj_key = object_name.lower()
        if "sports" in obj_key: obj_key = "sports" # Normalizzazione
        
        y_offset = 0.0
        if obj_key in TARGET_OFFSETS:
            y_offset = TARGET_OFFSETS[obj_key].get('pick_y_offset', 0.0)
            self.get_logger().info(f"DEBUG: Letto da config.py per '{obj_key}': pick_y_offset = {y_offset}")
            
        if y_offset != 0.0:
            final_pos[1] += y_offset
            self.get_logger().info(f"✨ Offset Y di {y_offset}m applicato per {object_name}")
        # -------------------------------------------------------------

        pose = PoseStamped()
        pose.header.frame_id = object_name
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = final_pos
        pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f"✅ [{side.upper()}] Trovato '{object_name}' a: {final_pos[0]:.3f}, {final_pos[1]:.3f}")
        
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
