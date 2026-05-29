#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import datetime
import os

class CameraRecorder(Node):
    def __init__(self):
        super().__init__('camera_recorder_node')
        
        # Parametri
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('output_dir', '/mm_ws/src/franka_bimanual_bringup/scripts/video')
        
        self.image_topic = self.get_parameter('image_topic').value
        self.fps = self.get_parameter('fps').value
        self.output_dir = self.get_parameter('output_dir').value
        
        # Setup
        self.bridge = CvBridge()
        self.is_recording = False
        self.video_writer = None
        
        # Sottoscrizione al topic della videocamera
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        # Servizio per avviare/fermare la registrazione (gestibile comodamente da rqt)
        self.srv = self.create_service(SetBool, '~/set_recording', self.set_recording_callback)
        
        self.get_logger().info(f'🎥 Nodo di registrazione pronto.')
        self.get_logger().info(f'Usa rqt (Service Caller) sul servizio {self.get_name()}/set_recording per avviare/fermare.')

    def image_callback(self, msg):
        if not self.is_recording:
            return
            
        try:
            # Converti il messaggio ROS in immagine OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Errore conversione immagine: {e}')
            return
            
        # Inizializza il video_writer al primo frame utile
        if self.video_writer is None:
            height, width, _ = cv_image.shape
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.output_dir, f'realsense_record_{timestamp}.mp4')
            
            # Formato MP4 con codec mp4v
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(filename, fourcc, self.fps, (width, height))
            self.get_logger().info(f'🔴 Registrazione AVVIATA. Salvataggio in: {filename}')
            
        # Scrivi il frame
        self.video_writer.write(cv_image)

    def set_recording_callback(self, request, response):
        if request.data and not self.is_recording:
            # START RECORDING
            self.is_recording = True
            response.success = True
            response.message = "Registrazione avviata."
            self.get_logger().info("Richiesta di avvio registrazione ricevuta.")
            
        elif not request.data and self.is_recording:
            # STOP RECORDING
            self.is_recording = False
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            response.success = True
            response.message = "Registrazione fermata."
            self.get_logger().info("⏹️ Registrazione FERMATA e file salvato con successo.")
            
        else:
            # Niente da fare
            response.success = True
            state = "già in corso" if self.is_recording else "già ferma"
            response.message = f"La registrazione era {state}."
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CameraRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.video_writer is not None:
            node.video_writer.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
