#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

class ForceCalibration(Node):
    def __init__(self):
        super().__init__('force_calibration')
        self.bridge = CvBridge()
        
        # Sottoscrizione
        self.sub_img = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_cb, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)
        self.intrinsics = None
        
        # Caricamento YOLO
        import onnxruntime as ort
        self.session = ort.InferenceSession('/mm_ws/yolo26s.onnx')
        self.get_logger().info("🎯 Calibrazione FORZATA in corso...")
        self.get_logger().info("Assicuratevi che la bottiglia sia al CENTRO (0,0,0)")

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
            if det[4] > 0.6:
                u, v = det[0], det[1] + (det[3] / 2)
                self.solve(u, v)
                break

    def solve(self, u, v):
        # 1. Posizione stimata (Quadrante Negativo)
        # Basandoci sui tuoi dati precedenti, la camera è circa qui:
        cam_x, cam_y, cam_z = -0.78, -1.17, 0.83
        
        # 2. Calcoliamo la rotazione per guardare il centro (0,0,0)
        # Z_axis punta verso il centro
        z_axis = np.array([0, 0, 0]) - np.array([cam_x, cam_y, cam_z])
        z_axis = z_axis / np.linalg.norm(z_axis)
        
        # Y_axis approssimativamente "giù" (rispetto al tavolo)
        # In frame camera optical, Y punta in basso. 
        # Cerchiamo un vettore X ortogonale a Z e alla verticale
        world_up = np.array([0, 0, 1])
        x_axis = np.cross(world_up, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        
        # Matrice di rotazione Optical to World
        R_opt_to_world = np.column_stack((x_axis, y_axis, z_axis))
        
        # Convertiamo in World (Body) frame
        # R_world = R_opt_to_world @ inv(R_body_to_optical)
        R_body_to_optical = Rotation.from_euler('xz', [-np.pi/2, -np.pi/2]).as_matrix()
        R_world_mat = R_opt_to_world @ np.linalg.inv(R_body_to_optical)
        
        rpy = Rotation.from_matrix(R_world_mat).as_euler('xyz')
        
        # 3. Salvataggio finale
        with open('/mm_ws/src/franka_bimanual_config/config/robot_poses.yaml', 'r') as f:
            config = yaml.safe_load(f)
        
        config['camera'] = {
            'x': float(cam_x), 'y': float(cam_y), 'z': float(cam_z),
            'roll': float(rpy[0]), 'pitch': float(rpy[1]), 'yaw': float(rpy[2])
        }
        
        with open('/mm_ws/src/franka_bimanual_config/config/robot_poses.yaml', 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
            
        print(f"\n✅ CALIBRAZIONE FORZATA COMPLETATA!")
        print(f"Posizione: x={cam_x}, y={cam_y}, z={cam_z}")
        print(f"Rotazione: roll={rpy[0]:.4f}, pitch={rpy[1]:.4f}, yaw={rpy[2]:.4f}")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ForceCalibration()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
