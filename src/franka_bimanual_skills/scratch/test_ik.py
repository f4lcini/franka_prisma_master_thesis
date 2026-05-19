import rclpy
from rclpy.node import Node
from franka_bimanual_skills.robot_control_api import RobotControlAPI
from geometry_msgs.msg import PoseStamped
import time

class IKTestNode(Node):
    def __init__(self):
        super().__init__('ik_test_node')
        self.api = RobotControlAPI(self, None)
        
    def run_test(self):
        time.sleep(2.0) # Attesa per connessione server
        
        # Definiamo una posa di test (es. 40cm davanti al robot destro)
        target = PoseStamped()
        target.header.frame_id = "table"
        target.pose.position.x = 0.4
        target.pose.position.y = -0.2
        target.pose.position.z = 0.2
        target.pose.orientation.x = 1.0 # Orientamento top-down
        target.pose.orientation.w = 0.0
        
        q = self.api.compute_ik("franka1_arm", target)
        
        if q:
            self.get_logger().info(f"✅ IK SUCCESSO! Configurazione giunti: \n{q}")
        else:
            self.get_logger().error("❌ IK FALLITA per questa posa.")

def main():
    rclpy.init()
    node = IKTestNode()
    node.run_test()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
