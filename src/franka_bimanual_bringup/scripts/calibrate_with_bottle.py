#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

class CalibrateWithBottle(Node):
    def __init__(self):
        super().__init__('calibrate_with_bottle')
        self.bridge = CvBridge()
        
        # Sottoscrizione
        self.sub_img = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_cb, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)
        self.intrinsics = None
        
        # Caricamento YOLO
        import onnxruntime as ort
        self.session = ort.InferenceSession('/mm_ws/yolo26s.onnx')
        self.get_logger().info("🚀 Mettere la bottiglia al CENTRO del tavolo (0,0,0) e attendere...")

    def info_cb(self, msg):
        self.intrinsics = {'fx': msg.k[0], 'fy': msg.k[4], 'cx': msg.k[2], 'cy': msg.k[5]}

    def image_cb(self, msg):
        if self.intrinsics is None: return
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img_resized = cv2.resize(cv_image, (640, 640))
        input_data = np.transpose(img_resized, (2, 0, 1)).astype(np.float32) / 255.0
        input_data = np.expand_dims(input_data, axis=0)
        
        outputs = self.session.run(None, {self.session.get_inputs()[0].name: input_data})
        detections = outputs[0][0]
        
        for det in detections:
            if det[4] > 0.6: # Fiducia alta
                # det è [cx, cy, w, h, score, class]
                u = det[0]
                v = det[1] + (det[3] / 2) # Usiamo la BASE della bottiglia (punto sul tavolo)
                self.calculate_and_save(u, v)
                break

    def calculate_and_save(self, u, v):
        # Assumiamo che la bottiglia sia a (0,0,0) nel mondo
        # Abbiamo bisogno di conoscere l'altezza della camera approssimativa (es. 0.83)
        # e l'orientamento (roll, pitch, yaw) che abbiamo calcolato prima con il tag
        # perché quello era abbastanza buono.
        
        with open('/mm_ws/src/franka_bimanual_config/config/robot_poses.yaml', 'r') as f:
            config = yaml.safe_load(f)
            cam = config['camera']
            roll, pitch, yaw = cam['roll'], cam['pitch'], cam['yaw']

        R_world = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        R_body_to_optical = Rotation.from_euler('xz', [-np.pi/2, -np.pi/2]).as_matrix()
        R_opt_to_world = R_world @ R_body_to_optical
        
        # Direzione raggio ottico
        v_opt = np.array([(u - self.intrinsics['cx']) / self.intrinsics['fx'], 
                         (v - self.intrinsics['cy']) / self.intrinsics['fy'], 1.0])
        v_world = R_opt_to_world @ v_opt
        
        # Sappiamo che P_world = cam_pos + lambda * v_world = [0, 0, 0]
        # Quindi cam_pos = -lambda * v_world
        # Ma sappiamo anche che cam_pos.z è circa 0.836
        # 0.836 = -lambda * v_world[2]  => lambda = -0.836 / v_world[2]
        
        z_fixed = 0.836 # Usiamo l'altezza del tag che era affidabile
        lam = -z_fixed / v_world[2]
        new_cam_pos = -lam * v_world
        # Forziamo Z
        new_cam_pos[2] = z_fixed

        print(f"\n✅ CALIBRAZIONE COMPLETATA!")
        print(f"Nuova Posizione Camera: x={new_cam_pos[0]:.4f}, y={new_cam_pos[1]:.4f}, z={new_cam_pos[2]:.4f}")
        
        cam.update({'x': float(new_cam_pos[0]), 'y': float(new_cam_pos[1]), 'z': float(new_cam_pos[2])})
        config['camera'] = cam
        
        with open('/mm_ws/src/franka_bimanual_config/config/robot_poses.yaml', 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
            
        print("File robot_poses.yaml aggiornato! Ora fai la build e testa.")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = CalibrateWithBottle()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
