import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from franka_custom_interfaces.action import DetectObject

import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

class ObjectLocalizationNode(Node):
    def __init__(self):
        super().__init__('object_localization_node')
        self.get_logger().info("Initializing Object Localization Node...")
        
        self.cv_bridge = CvBridge()
        self.latest_image = None
        self.latest_pointcloud = None
        self.latest_image_stamp = None
        self.latest_pc_stamp = None
        
        # Initialize YOLO. Use YOLOv8 nano for speed
        if YOLO is not None:
            self.get_logger().info("Loading YOLOv8 model...")
            self.model = YOLO('yolov8n.pt') 
        else:
            self.get_logger().error("ultralytics package is missing! Cannot run YOLO.")
            self.model = None

        self.cb_group = ReentrantCallbackGroup()

        # Camera topic subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10,
            callback_group=self.cb_group
        )
        
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points', 
            self.pc_callback,
            10,
            callback_group=self.cb_group
        )

        # Action Server
        self._action_server = ActionServer(
            self,
            DetectObject,
            'detect_object',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info("Object Localization Pipeline Ready. Waiting for goals on /detect_object")

    def image_callback(self, msg: Image):
        self.latest_image = msg
        self.latest_image_stamp = msg.header.stamp

    def pc_callback(self, msg: PointCloud2):
        self.latest_pointcloud = msg
        self.latest_pc_stamp = msg.header.stamp

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received detection goal for: '{goal_request.object_name}'")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for detection')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing localization pipeline...')
        feedback_msg = DetectObject.Feedback()
        result = DetectObject.Result()
        
        target_label = goal_handle.request.object_name.lower()
        
        if self.model is None:
            self.get_logger().error("YOLO model not loaded.")
            result.success = False
            result.error_message = "YOLO model not loaded."
            goal_handle.abort()
            return result
            
        if self.latest_image is None or self.latest_pointcloud is None:
            msg = "Missing sensory input (Image or PointCloud) from camera."
            self.get_logger().warn(msg)
            result.success = False
            result.error_message = msg
            goal_handle.abort()
            return result

        feedback_msg.status = "Analyzing image with YOLO..."
        goal_handle.publish_feedback(feedback_msg)

        try:
            # 1. YOLO Detection (2D Bounding Box)
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            frame_header = self.latest_image.header
            
            # Run inference
            results = self.model(cv_image, verbose=False)
            
            best_det = None
            best_conf = -1.0
            
            # YOLO results parsing
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    label = self.model.names[cls_id].lower()
                    
                    if label == target_label and conf > best_conf:
                        best_conf = conf
                        best_det = box.xyxy[0].cpu().numpy() # [x1, y1, x2, y2]
                        
            if best_det is None:
                msg = f"Target '{target_label}' not found in the current camera frame."
                self.get_logger().info(msg)
                result.success = False
                result.message = msg
                goal_handle.succeed() 
                return result
                
            x1, y1, x2, y2 = best_det
            u = int((x1 + x2) / 2.0)
            v = int((y1 + y2) / 2.0)
            
            self.get_logger().info(f"Found {target_label} at 2D pixel: ({u}, {v})")
            feedback_msg.status = f"Object 2D box found at ({u},{v}). Deprojecting..."
            goal_handle.publish_feedback(feedback_msg)
            
            # 2. 3D Deprojection (Pixel to Meters)
            pc_msg = self.latest_pointcloud
            
            # Extract point at (u, v)
            pc_data = list(pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=False, uvs=[[u, v]]))
            
            if not pc_data:
                raise Exception("Failed to read point from PointCloud")
                
            x, y, z = pc_data[0]
            
            # Neighborhood search if center pixel is NaN
            if np.isnan(x) or np.isnan(y) or np.isnan(z):
                found_valid = False
                for r in range(1, 10):
                    neighbors = [ [u+i, v+j] for i in range(-r, r+1) for j in range(-r, r+1) ]
                    # Filter for boundaries
                    valid_uvs = [uv for uv in neighbors if 0 <= uv[0] < pc_msg.width and 0 <= uv[1] < pc_msg.height]
                    if not valid_uvs: continue
                    points = list(pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=False, uvs=valid_uvs))
                    for pt in points:
                        if not np.isnan(pt[0]):
                            x, y, z = pt
                            found_valid = True
                            self.get_logger().info(f"Using valid depth neighbor around ({u},{v})")
                            break
                    if found_valid:
                        break
                        
                if not found_valid:
                    result.success = False
                    result.error_message = "Depth is NaN at object center and bounding box neighborhood."
                    goal_handle.abort()
                    return result
            
            # 3. Create PoseStamped Output for Motion Planner
            feedback_msg.status = "Pose computed. Returning result."
            goal_handle.publish_feedback(feedback_msg)
            
            target_pose = PoseStamped()
            target_pose.header.frame_id = pc_msg.header.frame_id
            target_pose.header.stamp = frame_header.stamp
            target_pose.pose.position.x = float(x)
            target_pose.pose.position.y = float(y)
            target_pose.pose.position.z = float(z)
            target_pose.pose.orientation.w = 1.0 
            
            result.success = True
            result.target_pose = target_pose
            result.message = f"Found '{target_label}' at (X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f})"
            
            goal_handle.succeed()
            self.get_logger().info(result.message)
            return result
            
        except Exception as e:
            self.get_logger().error(f"Localization Error: {e}")
            result.success = False
            result.error_message = str(e)
            goal_handle.abort()
            return result

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizationNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
