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
from franka_custom_interfaces.srv import ScanTable

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import datetime
import os
import time
import asyncio
import json
from scipy.spatial.transform import Rotation

# Abbassa la priorità di scheduling del processo YOLO rispetto al RT controller
try:
    os.nice(10)  # 0=normale, 19=minima priorità
except Exception:
    pass

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
        self.cached_scene = []

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

        # --- /scan_table Service: global table inventory for VLM grounding ---
        self.scan_table_server = self.create_service(
            ScanTable, 'scan_table',
            self.scan_table_callback,
            callback_group=self.action_cb_group
        )
        
        # Timer per il debug live (1 Hz - ridotto per non interferire con il RT controller)
        self.create_timer(1.0, self.debug_timer_callback, callback_group=self.sensor_cb_group)
        self._frame_counter = 0
        self._busy = False  # Flag: True mentre si esegue un pick/place

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
        """Esegue l'inferenza di debug a bassa frequenza (1Hz) e bassa risoluzione."""
        if self.latest_image is None or self.model is None:
            return
        # Non disturbare durante pick/place attivi
        if self._busy:
            return
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
            # Risoluzione ridotta (224px) per minimizzare il carico CPU
            results = self.model(cv_image, verbose=False, conf=0.3, imgsz=224)
            annotated_frame = results[0].plot(labels=True, boxes=True)
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
        self._busy = True
        result = DetectObject.Result()
        object_name = goal_handle.request.object_name
        self._cam_pos, self._R_optical_to_table = self._build_camera_transform()
        
        self.get_logger().info(f"🔍 [{side.upper()}] Cerco '{object_name}' nel mio spazio di lavoro...")

        # 1. Tentativo dalla Cache Globale (se disponibile)
        samples = []
        if getattr(self, 'cached_scene', []):
            self.get_logger().info(f"🔍 [{side.upper()}] Controllo la cache della scansione iniziale per '{object_name}'...")
            candidates_cache = []
            for det in self.cached_scene:
                if object_name.lower() in det['label'].lower():
                    p_table = np.array([det['x_world'], det['y_world'], 0.0])
                    candidates_cache.append((p_table, det['conf'], det))
                    
            # Filtro spaziale sulla cache
            if side == "left":
                valid_cache = [c for c in candidates_cache if c[0][0] < 0.05]
            else:
                valid_cache = [c for c in candidates_cache if c[0][0] >= -0.05]
                
            if valid_cache:
                best_cache = max(valid_cache, key=lambda x: x[1])
                samples.append(best_cache[0])
                self.cached_scene.remove(best_cache[2])  # Rimuovi per non riprenderlo!
                self.get_logger().info(f"✅ Trovato '{object_name}' in cache a X={best_cache[0][0]:.3f}!")

        # 2. Fallback a Scansione Live (se la cache è vuota o l'oggetto non c'è)
        if not samples:
            self.get_logger().info(f"⚠️ Nessun '{object_name}' in cache. Procedo con la scansione LIVE...")
            for _ in range(self.num_samples):
                if self.latest_image is None or self.camera_intrinsics is None:
                    time.sleep(0.1)
                    continue
                
                cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
                yolo_results = self.model(cv_image, verbose=False, imgsz=640)
                
                candidates = []
                for box in yolo_results[0].boxes:
                    conf = float(box.conf[0].item())
                    if object_name.lower() in self.model.names[int(box.cls[0].item())].lower() and conf > 0.10:
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        u, v = (x1 + x2) / 2.0, y2
                        v_opt = np.array([(u - self.camera_intrinsics['cx']) / self.camera_intrinsics['fx'], 
                                         (v - self.camera_intrinsics['cy']) / self.camera_intrinsics['fy'], 1.0])
                        v_table = self._R_optical_to_table @ v_opt
                        lam = -self._cam_pos[2] / v_table[2]
                        p_table = self._cam_pos + lam * v_table
                        candidates.append((p_table, conf))
    
                if not candidates and yolo_results[0].boxes:
                    seen = [f"{self.model.names[int(b.cls[0].item())]} ({float(b.conf[0].item()):.2f})" for b in yolo_results[0].boxes]
                    self.get_logger().info(f"   [DEBUG YOLO] Sto cercando '{object_name}' ma ho visto: {', '.join(seen)}")
                    
                if side == "left":
                    valid = [c for c in candidates if c[0][0] < 0.05]
                else:
                    valid = [c for c in candidates if c[0][0] >= -0.05]
    
                if candidates:
                    self.get_logger().info(f"🔍 [{side.upper()}] {len(candidates)} oggetti rilevati. Validi per questo lato: {len(valid)}")
                    for idx, c in enumerate(candidates):
                        status = "VALIDO" if (side == "left" and c[0][0] < 0.05) or (side == "right" and c[0][0] >= -0.05) else "FUORI_ZONA"
                        self.get_logger().info(f"   -> [{status}] X={c[0][0]:.3f}, Conf={c[1]:.2f}")
    
                if valid:
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
        self._busy = False
        return result

    def scan_table_callback(self, request, response):
        """
        /scan_table service handler.
        Scans the full table with YOLO (3 frames, both sides), applies NMS
        and returns all unique detected objects as a JSON string.
        Format: [{"label", "x_world", "y_world", "side", "conf"}, ...]
        """
        self.get_logger().info("🔍 /scan_table: avvio scan globale del tavolo...")

        response.success = False
        response.scene_json = "[]"

        if self.model is None:
            response.message = "YOLO model not loaded."
            return response
        if self.latest_image is None or self.camera_intrinsics is None:
            response.message = "Camera not ready (no image or intrinsics)."
            return response

        # Rebuild camera transform in case parameters changed at runtime
        cam_pos, R_opt_to_table = self._build_camera_transform()

        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        cx = self.camera_intrinsics['cx']
        cy = self.camera_intrinsics['cy']

        NUM_FRAMES  = 3     # campioni per robustezza temporale
        NMS_DIST_M  = 0.08  # soglia NMS: oggetti a <8 cm = stesso oggetto

        all_detections = []

        for _ in range(NUM_FRAMES):
            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
                results  = self.model(cv_image, verbose=False, conf=0.10, imgsz=640)

                for box in results[0].boxes:
                    cls_id = int(box.cls[0].item())
                    conf   = float(box.conf[0].item())
                    label  = self.model.names[cls_id]

                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    u = (x1 + x2) / 2.0
                    v = y2  # base del bounding box = punto di appoggio

                    # Proiezione sul piano tavolo (z=0 nel frame mondo)
                    v_opt   = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
                    v_world = R_opt_to_table @ v_opt
                    if abs(v_world[2]) < 1e-6:
                        continue
                    lam     = -cam_pos[2] / v_world[2]
                    p_world = cam_pos + lam * v_world

                    all_detections.append({
                        'label':   label,
                        'x_world': float(p_world[0]),
                        'y_world': float(p_world[1]),
                        'conf':    conf,
                    })
            except Exception as e:
                self.get_logger().error(f"Errore YOLO in scan_table frame: {e}")
            time.sleep(0.05)

        if not all_detections:
            response.success = True
            response.scene_json = "[]"
            response.message = "No objects detected on the table."
            self.get_logger().info("🔍 /scan_table: nessun oggetto rilevato.")
            return response

        # NMS spaziale: per ogni (label, posizione XY) vicini, tieni solo best-conf
        kept = []
        for det in sorted(all_detections, key=lambda d: -d['conf']):
            duplicate = False
            for k in kept:
                dist = ((k['x_world'] - det['x_world'])**2 +
                        (k['y_world'] - det['y_world'])**2) ** 0.5
                if k['label'] == det['label'] and dist < NMS_DIST_M:
                    duplicate = True
                    break
            if not duplicate:
                det['side'] = "left_side" if det['x_world'] < 0.0 else "right_side"
                kept.append(det)

        self.get_logger().info(
            f"✅ /scan_table: {len(all_detections)} raw det → {len(kept)} oggetti unici"
        )
        for obj in kept:
            self.get_logger().info(
                f"   · '{obj['label']}' @ X={obj['x_world']:.3f}m → {obj['side']} (conf={obj['conf']:.2f})"
            )

        # Salva in cache globale
        self.cached_scene = kept

        response.success   = True
        response.scene_json = json.dumps(kept)
        response.message   = f"Found {len(kept)} unique object(s) on the table."
        return response


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
