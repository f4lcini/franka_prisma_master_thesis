#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_msgs.srv import GetPositionIK
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

class VisualHandoverOptimizer(Node):
    def __init__(self):
        super().__init__('visual_handover_optimizer')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
            
        self.get_logger().info('✅ Service ready. Starting Bimanual Mathematical Analysis...')
        
        # Limiti hardware del Panda Franka (approssimati per sicurezza)
        self.joint_limits = {
            'panda_joint1': (-2.8973, 2.8973),
            'panda_joint2': (-1.7628, 1.7628),
            'panda_joint3': (-2.8973, 2.8973),
            'panda_joint4': (-3.0718, -0.0698),
            'panda_joint5': (-2.8973, 2.8973),
            'panda_joint6': (-0.0175, 3.7525),
            'panda_joint7': (-2.8973, 2.8973)
        }

    def apply_donor_orientation(self, pose: PoseStamped):
        """Right arm points towards +Y (face-to-face)."""
        pose.pose.orientation = Quaternion(x=-0.7071, y=0.0, z=0.0, w=0.7071)

    def apply_recipient_orientation(self, pose: PoseStamped):
        """Left arm points towards -Y (symmetrical face) with 90 deg finger offset."""
        pose.pose.orientation = Quaternion(x=0.5, y=0.5, z=0.5, w=0.5)

    def compute_joint_centering_penalty(self, joint_names, joint_positions):
        """
        Calcola quanto la configurazione è 'rilassata'.
        Più si avvicina a 0, più i giunti sono lontani dai loro limiti fisici (massima destrezza).
        """
        penalty = 0.0
        # Compute penalty based on distance from center for joints 1-7
        for name, pos in zip(joint_names, joint_positions):
            # Normalizza ai joint del franka scartando il prefisso e il dito
            j_name = "panda_" + name.split("_")[-1]
            if j_name in self.joint_limits:
                min_l, max_l = self.joint_limits[j_name]
                center = (max_l + min_l) / 2.0
                range_l = max_l - min_l
                
                # Normalised distance from center [-1, 1]
                norm_dist = (pos - center) / (range_l / 2.0)
                # Funzione di costo quadratica (penalizza fortemente gli estremi)
                penalty += norm_dist ** 2
        return penalty

    def query_ik(self, group_name, pose_stamped):
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 20000000 
        
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def generate_heatmap(self):
        # Griglia Spaziale (Asse X e Z)
        x_min, x_max = 0.40, 0.90
        z_min, z_max = 0.15, 0.65
        step = 0.02 # Alta risoluzione (2 cm)
        
        y_fixed = 0.587 
        
        X_coords = np.arange(x_min, x_max + step, step)
        Z_coords = np.arange(z_min, z_max + step, step)
        
        # Matrice dei risultati (Inizializzata a NaN = Spazio Irraggiungibile)
        heatmap_data = np.full((len(Z_coords), len(X_coords)), np.nan)
        
        best_score = float('inf')
        best_coord = (0, 0)
        
        self.get_logger().info("Eseguendo analisi spaziale (Face-to-Face). Attendi...\n")
        
        for z_idx, z in enumerate(reversed(Z_coords)): # Reversed to match matrix display
            for x_idx, x in enumerate(X_coords):
                # 1. Test Recipient (Left) - Approach & Target
                p_left_app = PoseStamped()
                p_left_app.header.frame_id = "world"
                p_left_app.pose.position = Point(x=x, y=y_fixed + 0.10, z=z)
                self.apply_recipient_orientation(p_left_app)
                res_l_app = self.query_ik("franka2_manipulator", p_left_app)
                
                p_left_tgt = PoseStamped()
                p_left_tgt.header.frame_id = "world"
                p_left_tgt.pose.position = Point(x=x, y=y_fixed, z=z)
                self.apply_recipient_orientation(p_left_tgt)
                res_l_tgt = self.query_ik("franka2_manipulator", p_left_tgt)
                
                # 2. Test Donor (Right) - Approach & Target
                p_right_app = PoseStamped()
                p_right_app.header.frame_id = "world"
                p_right_app.pose.position = Point(x=x, y=y_fixed - 0.10, z=z)
                self.apply_donor_orientation(p_right_app)
                res_r_app = self.query_ik("franka1_manipulator", p_right_app)
                
                p_right_tgt = PoseStamped()
                p_right_tgt.header.frame_id = "world"
                p_right_tgt.pose.position = Point(x=x, y=y_fixed, z=z)
                self.apply_donor_orientation(p_right_tgt)
                res_r_tgt = self.query_ik("franka1_manipulator", p_right_tgt)
                
                # If ALL states are feasible
                if all(res.error_code.val == 1 for res in [res_l_app, res_l_tgt, res_r_app, res_r_tgt]):
                    # Score based on Target poses (the most stressed configuration)
                    pen_l = self.compute_joint_centering_penalty(res_l_tgt.solution.joint_state.name, res_l_tgt.solution.joint_state.position)
                    pen_r = self.compute_joint_centering_penalty(res_r_tgt.solution.joint_state.name, res_r_tgt.solution.joint_state.position)
                    total_penalty = pen_l + pen_r
                    
                    heatmap_data[z_idx, x_idx] = total_penalty
                    
                    if total_penalty < best_score:
                        best_score = total_penalty
                        best_coord = (x, z)
                        best_joints_l = res_l_tgt.solution.joint_state.position
                        best_joints_r = res_r_tgt.solution.joint_state.position
        
        if not np.all(np.isnan(heatmap_data)):
            self.get_logger().info(f"\n🏆 PUNTO IDEALE OTTENUTO!\nCoordinate: X={best_coord[0]:.3f}, Z={best_coord[1]:.3f}")
            self.get_logger().info(f"Joints Left (franka2): {['%.4f' % j for j in best_joints_l]}")
            self.get_logger().info(f"Joints Right (franka1): {['%.4f' % j for j in best_joints_r]}\n")
        else:
            self.get_logger().error("❌ NESSUN PUNTO DI INTERSEZIONE TROVATO NEI LIMITI SPECIFICATI.")
        
        self.get_logger().info("📈 Sto generando il materiale visivo per la tesi...")
        # ... Rest of the plot code stays the same ...
        plt.figure(figsize=(10, 8))
        sns.heatmap(heatmap_data, 
                    xticklabels=np.round(X_coords, 2), 
                    yticklabels=np.round(Z_coords[::-1], 2), 
                    cmap="viridis_r",
                    cbar_kws={'label': 'Joint Centering Penalty (Lower is Better)'})
        plt.title('Bimanual Handover Workspace Analysis (Symmetrical Approach)', fontsize=16)
        plt.xlabel('X Coordinate (m)', fontsize=14)
        plt.ylabel('Z Coordinate (m)', fontsize=14)
        file_path = "/mm_ws/bimanual_handover_heatmap.png"
        plt.savefig(file_path, dpi=300, bbox_inches='tight')
        self.get_logger().info(f"✅ Immagine generata e salvata in: {file_path}")

def main(args=None):
    rclpy.init(args=args)
    node = VisualHandoverOptimizer()
    node.generate_heatmap()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
