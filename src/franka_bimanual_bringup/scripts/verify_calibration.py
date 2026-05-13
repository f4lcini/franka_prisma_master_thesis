#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

class VerifyCalibration(Node):
    def __init__(self):
        super().__init__('verify_calibration')
        self.bridge = CvBridge()
        
        # Carica parametri camera da robot_poses.yaml
        self.declare_parameter('poses_path', '/mm_ws/src/franka_bimanual_config/config/robot_poses.yaml')
        path = self.get_parameter('poses_path').value
        with open(path, 'r') as f:
            config = yaml.safe_load(f)
            cam = config['camera']
            self.cam_pos = np.array([cam['x'], cam['y'], cam['z']])
            self.rpy = [cam['roll'], cam['pitch'], cam['yaw']]
            self.R_world = Rotation.from_euler('xyz', self.rpy).as_matrix()
            
            # Matrice Optical to Table con flip asse X (corregge inversione sistematica)
            R_body_to_optical = Rotation.from_euler('xz', [-np.pi/2, -np.pi/2]).as_matrix()
            X_flip = np.diag([-1.0, 1.0, 1.0])
            self.R_opt_to_table = self.R_world @ R_body_to_optical @ X_flip

        self.get_logger().info(f"✅ VERIFICATORE PRONTO")
        self.get_logger().info(f"Camera caricata da YAML: POS={self.cam_pos} RPY={self.rpy}")

        # Sottoscrizione
        self.sub_img = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_cb, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)
        self.intrinsics = None
        
        import onnxruntime as ort
        self.session = ort.InferenceSession('/mm_ws/yolo26s.onnx')

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
            if det[4] > 0.5:
                # Usiamo la BASE per coerenza con la calibrazione
                u = det[0]
                v = det[1] + (det[3] / 2)
                
                v_opt = np.array([(u - self.intrinsics['cx']) / self.intrinsics['fx'], 
                                 (v - self.intrinsics['cy']) / self.intrinsics['fy'], 1.0])
                v_table = self.R_opt_to_table @ v_opt
                
                if abs(v_table[2]) > 0.001:
                    lam = -self.cam_pos[2] / v_table[2]
                    P_table = self.cam_pos + lam * v_table
                    print(f"\r📍 BOTTIGLIA (BASE) -> X: {P_table[0]:.3f} | Y: {P_table[1]:.3f} | Z: 0.000", end="")

def main():
    rclpy.init()
    node = VerifyCalibration()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
