#!/usr/bin/env python3
"""
Verifica la calibrazione usando l'AprilTag (nessun YOLO).
Il tag deve essere al CENTRO del tavolo (0,0,0).
Se la calibrazione è corretta: TAG RILEVATO -> X: ~0.000 | Y: ~0.000 | Z: ~0.000
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

class VerifyCalibrationApriltag(Node):
    def __init__(self):
        super().__init__('verify_calibration_apriltag')
        
        # Carica camera dal YAML
        path = '/mm_ws/src/franka_bimanual_config/config/robot_poses.yaml'
        with open(path, 'r') as f:
            config = yaml.safe_load(f)
            cam = config['camera']
            self.cam_pos = np.array([cam['x'], cam['y'], cam['z']])
            self.rpy = [cam['roll'], cam['pitch'], cam['yaw']]
            R_body = Rotation.from_euler('xyz', self.rpy).as_matrix()
            R_body_to_optical = Rotation.from_euler('xz', [-np.pi/2, -np.pi/2]).as_matrix()
            self.R_opt_to_table = R_body @ R_body_to_optical

        self.get_logger().info(f"✅ Camera: pos={self.cam_pos.round(3)} rpy={[round(r,3) for r in self.rpy]}")
        self.get_logger().info("📌 Tag deve essere al CENTRO del tavolo (0,0,0)")
        self.get_logger().info("📌 Risultato atteso: X≈0, Y≈0, Z≈0")
        self.get_logger().info("-" * 50)

        # Legge il tf dell'apriltag da /tf e /tf_static
        self.sub_tf = self.create_subscription(TFMessage, '/tf', self.tf_cb, 10)
        self.sub_tf_static = self.create_subscription(TFMessage, '/tf_static', self.tf_cb, 10)
        self.seen_frames = set()
        self.create_timer(3.0, self.print_seen_frames)

    def print_seen_frames(self):
        if self.seen_frames:
            self.get_logger().info(f"Frame visti finora: {sorted(self.seen_frames)}")
        else:
            self.get_logger().warn("Nessun frame ricevuto ancora. Apriltag_node è attivo?")

    def tf_cb(self, msg: TFMessage):
        for transform in msg.transforms:
            frame = transform.child_frame_id
            self.seen_frames.add(frame)
            
            # Cerca il frame del tag (tag_0 o varianti)
            if 'tag' in frame.lower():
                t = transform.transform.translation
                t_opt = np.array([t.x, t.y, t.z])
                P_table = self.cam_pos + self.R_opt_to_table @ t_opt
                print(f"\r📍 [{frame}] -> X: {P_table[0]:.4f} | Y: {P_table[1]:.4f} | Z: {P_table[2]:.4f}", end="")

def main():
    rclpy.init()
    node = VerifyCalibrationApriltag()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
