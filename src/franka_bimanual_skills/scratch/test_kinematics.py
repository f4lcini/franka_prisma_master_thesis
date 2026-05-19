import rclpy
from rclpy.node import Node
from franka_bimanual_skills.robot_control_api import RobotControlAPI
from geometry_msgs.msg import PoseStamped
import time

class KinematicsTestNode(Node):
    def __init__(self):
        super().__init__('kinematics_test_node')
        self.api = RobotControlAPI(self, None)
        
    def run_test(self):
        time.sleep(2.0)
        
        # 1. TEST INVERSE KINEMATICS (IK)
        target = PoseStamped()
        target.header.frame_id = "table"
        target.pose.position.x = 0.5
        target.pose.position.y = -0.1
        target.pose.position.z = 0.3
        target.pose.orientation.x = 1.0 
        target.pose.orientation.w = 0.0
        
        self.get_logger().info("--- TEST 1: IK (Cartesian -> Joints) ---")
        q = self.api.compute_ik("franka1_arm", target)
        
        if q:
            self.get_logger().info(f"✅ IK SUCCESSO! Giunti: {q}")
            
            # 2. TEST FORWARD KINEMATICS (FK)
            self.get_logger().info("--- TEST 2: FK (Joints -> Cartesian) ---")
            pose_res = self.api.compute_fk("franka1_arm", q)
            
            if pose_res:
                pos = pose_res.pose.position
                self.get_logger().info(f"✅ FK SUCCESSO! Il TCP si trova a:")
                self.get_logger().info(f"   X: {pos.x:.3f}, Y: {pos.y:.3f}, Z: {pos.z:.3f}")
        else:
            self.get_logger().error("❌ IK FALLITA.")

def main():
    rclpy.init()
    node = KinematicsTestNode()
    node.run_test()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
