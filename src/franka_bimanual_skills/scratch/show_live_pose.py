import rclpy
from rclpy.node import Node
from franka_bimanual_skills.robot_control_api import RobotControlAPI
import time
import os

class LiveDashboardNode(Node):
    def __init__(self):
        super().__init__('live_dashboard_node')
        self.api = RobotControlAPI(self, None)
        
    def show_dashboard(self):
        self.get_logger().info("📡 Avvio Dashboard Live... (Ctrl+C per uscire)")
        time.sleep(2.0)
        
        while rclpy.ok():
            os.system('clear') # Pulisce lo schermo per un effetto dashboard
            print("================================================================")
            print("         BIMANUAL FRANKA LIVE KINEMATICS DASHBOARD")
            print("================================================================")
            
            for arm_name, prefix in [("RIGHT ARM (Franka1)", "franka1"), ("LEFT ARM (Franka2)", "franka2")]:
                # 1. Recupero Giunti
                joints_names = [f"{prefix}_fr3_joint{i}" for i in range(1, 8)]
                q_live = []
                for j in joints_names:
                    if j in self.api.current_joints:
                        q_live.append(self.api.current_joints[j])
                
                print(f"\n🤖 {arm_name}:")
                if len(q_live) == 7:
                    # Stampa Giunti (q)
                    q_str = ", ".join([f"{v:.2f}" for v in q_live])
                    print(f"   [GIUNTI q]: {q_str}")
                    
                    # 2. Calcolo FK (Cartesiana)
                    pose_res = self.api.compute_fk(f"{prefix}_arm", q_live)
                    if pose_res:
                        p = pose_res.pose.position
                        print(f"   [CARTESIA]: X={p.x:.3f}, Y={p.y:.3f}, Z={p.z:.3f}")
                    else:
                        print(f"   [CARTESIA]: Calcolo in corso...")
                else:
                    print(f"   [DATI]: In attesa dei sensori...")
            
            print("\n================================================================")
            print(" [INFO]: Muovi i robot per vedere i valori cambiare.")
            
            # Aggiornamento ROS
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

def main():
    rclpy.init()
    node = LiveDashboardNode()
    try:
        node.show_dashboard()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
