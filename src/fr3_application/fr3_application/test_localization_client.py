import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_custom_interfaces.action import DetectObject
import sys

class TestLocalizationClient(Node):
    def __init__(self):
        super().__init__('test_localization_client')
        self._action_client = ActionClient(self, DetectObject, 'detect_object')

    def send_goal(self, object_name):
        goal_msg = DetectObject.Goal()
        goal_msg.object_name = object_name

        self.get_logger().info(f"Waiting for action server 'detect_object'...")
        self._action_client.wait_for_server()
        
        self.get_logger().info(f"Sending goal to locate '{object_name}'...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.status}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by the Action Server.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted! Processing...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result Success: {result.success}")
        if result.success:
            p = result.target_pose.pose.position
            self.get_logger().info(f"Object Pose [meters]: X={p.x:.3f}, Y={p.y:.3f}, Z={p.z:.3f}")
            self.get_logger().info(f"Frame ID: {result.target_pose.header.frame_id}")
        else:
            self.get_logger().error(f"Detection Failed. Message: {result.message}")
            if result.error_message:
                self.get_logger().error(f"Error Details: {result.error_message}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = TestLocalizationClient()
    
    target = 'bottle'
    if len(sys.argv) > 1:
        target = sys.argv[1]
        
    action_client.send_goal(target)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
