"""
================================================================================
Author: Falco Robotics 
Code Description:
This ROS 2 node performs 3D object localization by fusing 2D visual object 
detection (using YOLOv8) with 3D depth data (from a PointCloud2 topic). 
It exposes a ROS 2 Action Server that listens for a target object name, locates 
it in the camera's RGB stream, deprojects the 2D bounding box center to a 3D 
point using the depth map, and returns the real-world 3D pose of the object.


Pipeline: Perception

Implementation Steps Summary:
- NODE INITIALIZATION (Steps 1-4): Setup ROS logging, CvBridge, variable buffers, and YOLO model.
- TOPIC SUBSCRIBERS (Steps 5-7): Configure Reentrant callback group and subscribe to RGB and Depth topics.
- ACTION SERVER (Step 8): Setup Server to handle incoming 'detect_object' tasks.
- SENSOR CALLBACKS (Steps 9-10): Buffer incoming Image and PointCloud2 payloads.
- ACTION LIFECYCLE (Steps 11-12): Handle goal acceptance and explicit cancellation requests.
- EXECUTION START (Steps 13-15): Validate prerequisites and extract the target object name.
- 2D DETECTION (Steps 16-19): Convert image to OpenCV, run YOLO inference, parse results, and extract the bounding box center pixel.
- 3D DEPROJECTION (Steps 20-21): Query the PointCloud at the center pixel, deploying a fail-safe spatial search if NaN.
- RESULT FINALIZATION (Steps 22-23): Wrap the 3D coordinates into a PoseStamped message and conclude the action successfully.
- MAIN LOOP & EXECUTOR (Steps 24-28): Initialize rclpy, spin the node in a MultiThreadedExecutor, and shutdown gracefully.
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from franka_custom_interfaces.action import DetectObject

import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class ObjectLocalizationNode(Node):
    def __init__(self):
        super().__init__('object_localization_node')
        
        # Step 1: Print an initialization log message.
        self.get_logger().info('Object Localization Node Initializing...')
        
        # Step 2: Initialize a CvBridge object to convert ROS images to OpenCV format for YOLO processing.
        self.cv_bridge = CvBridge()
        
        # Step 3: Create internal variables to buffer the latest incoming sensor data.
        self.latest_image = None
        self.latest_image_time = None
        self.latest_pointcloud = None
        self.latest_pointcloud_time = None

        # Step 4: Initialize the YOLOv8 model. Handle the case where the ultralytics package is missing.
        if YOLO is None:
            self.get_logger().error('YOLO algorithm is not installed.')
        else:
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info('YOLO algorithm loaded successfully.')
        
        # ------------- SENSOR SUBSCRIBERS -------------

        # Step 5: Create a ReentrantCallbackGroup to allow concurrent callback execution.
        self.reetrant_group = ReentrantCallbackGroup()
        
        # Step 6: Create a subscriber for the RGB camera image topic.
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10,
            callback_group=self.reetrant_group
        )

        # Step 7: Create a subscriber for the PointCloud depth topic.
        self.pc_subscriber = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.pc_callback,
            10,
            callback_group=self.reetrant_group
        )

        # ------------- ACTION SERVER -------------

        # Step 8: Initialize the Action Server to receive and process localization commands.
        self.detect_object_server = ActionServer(
            self,
            DetectObject,
            'detect_object',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.reetrant_group
        )

    # ------------- SENSOR CALLBACKS -------------

    def image_callback(self, msg: Image):
        # Step 9: Store the most recent RGB image and its timestamp.
        self.latest_image = msg
        self.latest_image_time = msg.header.stamp
        
    def pc_callback(self, msg: PointCloud2):
        # Step 10: Store the most recent PointCloud data and its timestamp.
        self.latest_pointcloud = msg
        self.latest_pointcloud_time = msg.header.stamp

    # ------------- ACTION SERVER CALLBACKS -------------

    def goal_callback(self, goal_request):
        # Step 11: Accept incoming goals and log the target object name requested by the client.
        self.get_logger().info(f'Goal received. Target object: {goal_request.object_name}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Step 12: Accept cancellation requests and log the event.
        self.get_logger().info('Goal explicitly cancelled by the client.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        # Step 13: Acknowledge the start of the action execution and initialize action message types.
        self.get_logger().info('Action execution started.')
        feedback_msg = DetectObject.Feedback()
        result_msg = DetectObject.Result()
                
        # Step 14: Verify node prerequisites before proceeding (YOLO readiness and sensor data availability).
        if YOLO is None or self.latest_image is None or self.latest_pointcloud is None:
            self.get_logger().error('Node not ready: missing YOLO, Image data, or PointCloud data.')
            result_msg.success = False
            goal_handle.abort()
            return result_msg
            
        # Step 15: Extract the requested object name from the goal handle payload.
        object_name = goal_handle.request.object_name
        self.get_logger().info(f'Commencing visual search for: {object_name}')

        # Step 16: Convert the stored ROS Image into an OpenCV format image for neural network processing.
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Translation Error: {e}')
            result_msg.success = False
            goal_handle.abort()
            return result_msg
            
        # Step 17: Execute the model inference passing the OpenCV image to the YOLO network.
        results = self.model(cv_image, verbose=False)
        
        # Step 18: Parse YOLO results to find the highest-confidence bounding box matching the requested object.
        best_box = None
        best_conf = 0.0
        
        for box in results[0].boxes:
            cls_id = int(box.cls[0].item())
            label = self.model.names[cls_id]
            conf = float(box.conf[0].item())
            
            if label == object_name and conf > best_conf:
                best_box = box
                best_conf = conf
                
        # Handle the edge case where the requested object is not found in the current frame.
        if best_box is None:
            self.get_logger().error(f'Target object "{object_name}" not detected in the current frame.')
            result_msg.success = False
            goal_handle.abort()
            return result_msg
        
        # Step 19: Calculate the 2D center point (u, v) in pixels from the resulting bounding box coordinates.
        x1, y1, x2, y2 = best_box.xyxy[0].tolist()
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)
        self.get_logger().info(f'Target "{object_name}" found physically at 2D pixel coordinates: (u:{u}, v:{v})')

        # Step 20: Cross-reference the 2D pixel center with the PointCloud depth map to retrieve the 3D coordinates.
        pc_msg = self.latest_pointcloud
        pc_data = list(pc2.read_points(pc_msg, skip_nans=False, field_names=("x", "y", "z"), uvs=[[u, v]]))
        
        if not pc_data:
            self.get_logger().error("Critical error while indexing PointCloud data at the specified pixel.")
            result_msg.success = False
            goal_handle.abort()
            return result_msg
            
        x, y, z = pc_data[0]

        # Step 21: Implement a fail-safe spatial search if the exact center pixel yields a NaN depth value.
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            self.get_logger().warn("Center pixel depth is NaN. Initiating neighborhood depth search...")
            found_valid = False
            search_radius_pixels = 10
            
            # Generate a localized grid of coordinate offsets around the bounding box center.
            offsets = [ [i, j] for i in range(-search_radius_pixels, search_radius_pixels+1) 
                               for j in range(-search_radius_pixels, search_radius_pixels+1) ]
            neighbors = [ [u+i, v+j] for i, j in offsets ]
            
            # Filter boundary-exceeding pixels.
            valid_uvs = [uv for uv in neighbors if 0 <= uv[0] < pc_msg.width and 0 <= uv[1] < pc_msg.height]
            
            if valid_uvs:
                points = list(pc2.read_points(pc_msg, skip_nans=False, field_names=("x", "y", "z"), uvs=valid_uvs))
                for pt in points:
                    if not np.isnan(pt[0]):
                        x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
                        found_valid = True
                        break
                        
            if not found_valid:
                self.get_logger().error("No valid 3D depth data found in the vicinity of the object.")
                result_msg.success = False
                result_msg.message = "Unable to compute 3D position (Depth is NaN)."
                goal_handle.abort()
                return result_msg
        else:
            x, y, z = float(x), float(y), float(z)

        # Step 22: Compile the output message payload structuring the 3D variables into a PoseStamped geometry.
        target_pose = PoseStamped()
        target_pose.header.frame_id = pc_msg.header.frame_id
        target_pose.header.stamp = self.latest_image_time
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.w = 1.0  # Default neutral rotation quaternion.
        
        # Step 23: Finalize the action server state machine, flag as succeeded, and return the result.
        result_msg.success = True
        result_msg.target_pose = target_pose
        result_msg.message = f"Successfully localized '{object_name}' at physical coordinates (X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f})"
        
        self.get_logger().info(result_msg.message)
        goal_handle.succeed()
        return result_msg


# ------------- MAIN FUNCTION -------------

def main(args=None):
    # Step 24: Initialize the rclpy middleware framework.
    rclpy.init(args=args)
    
    # Step 25: Instantiate the custom ObjectLocalizationNode object.
    node = ObjectLocalizationNode()
    
    # Step 26: Create a MultiThreadedExecutor to handle asynchronous subscription callbacks alongside Action Server routines.
    executor = MultiThreadedExecutor()
    
    # Step 27: Attach the node to the executor and keep spinning it to process incoming ROS messages indefinitely.
    executor.add_node(node)
    executor.spin()
    
    # Step 28: Gracefully dismantle the node and shut down the rclpy API upon termination.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
