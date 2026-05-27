#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from franka_custom_interfaces.srv import ScanTable
import cv2
from cv_bridge import CvBridge
import os
import argparse
import datetime
import json
from collections import defaultdict

class SnapshotNode(Node):
    def __init__(self, target_requirements):
        super().__init__('snapshot_node')
        
        # Dizionario con {nome_oggetto: quantità_minima}
        self.target_requirements = target_requirements
        self.cv_bridge = CvBridge()
        self.last_scan_image = None
        self.saved = False
        
        # Sottoscrizione all'immagine annotata prodotta dal server durante la scan
        self.create_subscription(Image, '/yolo_scan_image', self.image_callback, 10)
        
        # Client per avviare la scansione
        self.scan_client = self.create_client(ScanTable, 'scan_table')
        
        # Timer per eseguire la scansione finché non troviamo gli oggetti
        self.timer = self.create_timer(3.0, self.check_and_scan)
        
        req_str = ", ".join([f"{k} (x{v})" for k, v in self.target_requirements.items()])
        self.get_logger().info(f"📸 In attesa di trovare esattamente questi oggetti sul tavolo:")
        self.get_logger().info(f"   👉 {req_str}")
        self.get_logger().info("Farò una foto appena saranno TUTTI rilevati!")
        
    def image_callback(self, msg):
        self.last_scan_image = msg
        
    def check_and_scan(self):
        if self.saved:
            return
            
        if not self.scan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('In attesa del servizio /scan_table del server di percezione...')
            return
            
        self.get_logger().info('Avvio scansione tavolo...')
        req = ScanTable.Request()
        future = self.scan_client.call_async(req)
        future.add_done_callback(self.scan_done_callback)
        
    def scan_done_callback(self, future):
        if self.saved: return
        try:
            response = future.result()
            if response.success:
                scene = json.loads(response.scene_json)
                found_counts = defaultdict(int)
                
                # Conta le occorrenze di ogni oggetto target rilevato
                for obj in scene:
                    for target in self.target_requirements.keys():
                        if target.lower() in obj['label'].lower():
                            found_counts[target] += 1
                
                # Verifica se TUTTI i requisiti sono stati soddisfatti
                requirements_met = True
                for target, required_count in self.target_requirements.items():
                    if found_counts[target] < required_count:
                        requirements_met = False
                        break
                
                # Stampa lo stato attuale per debug
                status_str = ", ".join([f"{k}: {found_counts[k]}/{v}" for k, v in self.target_requirements.items()])
                self.get_logger().info(f"📊 Stato rilevamento: {status_str}")

                if requirements_met:
                    self.get_logger().info("🎯 Tutti i requisiti soddisfatti! Preparo la foto...")
                    
                    if self.last_scan_image is not None:
                        # Salva l'immagine annotata
                        cv_image = self.cv_bridge.imgmsg_to_cv2(self.last_scan_image, 'bgr8')
                        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                        
                        # Percorso in cui salvare le foto
                        save_dir = os.path.join(
                            os.path.dirname(os.path.abspath(__file__)), 
                            "experiment_logs/Yolo_boundingboxes"
                        )
                        os.makedirs(save_dir, exist_ok=True)
                        
                        filename = f"snapshot_{timestamp}.jpg"
                        filepath = os.path.join(save_dir, filename)
                        cv2.imwrite(filepath, cv_image)
                        
                        self.get_logger().info(f"✅ Foto con bounding boxes salvata in: {filepath}")
                        self.saved = True
                        
                        # Esci dallo script dopo aver fatto la foto
                        raise SystemExit
                    else:
                        self.get_logger().warn("Nessuna immagine /yolo_scan_image ricevuta dal server. Riprovo al prossimo ciclo.")
                else:
                    self.get_logger().info("Non ci sono ancora tutti gli oggetti richiesti. Riprovo tra poco...")
        except SystemExit:
            raise
        except Exception as e:
            self.get_logger().error(f"Errore durante l'elaborazione della risposta: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description="Scatta una foto quando rileva determinati oggetti nelle quantità specificate.")
    parser.add_argument(
        '--objects', 
        nargs='+', 
        help="Lista di oggetti e quantità. Es: --objects apple=2 banana=1 'sports ball'=1", 
        required=True
    )
    
    # Rimuoviamo gli argomenti ROS (se presenti) per non far confondere argparse
    ros_args = []
    script_args = []
    import sys
    for arg in sys.argv[1:]:
        if arg.startswith('__node:=') or arg.startswith('__log_disable_rosout:='):
            ros_args.append(arg)
        else:
            script_args.append(arg)
            
    parsed_args = parser.parse_args(script_args)
    
    target_requirements = {}
    for item in parsed_args.objects:
        if '=' in item:
            name, count = item.split('=', 1)
            try:
                target_requirements[name.lower().strip()] = int(count)
            except ValueError:
                print(f"Errore: '{count}' non è un numero valido per la quantità.")
                sys.exit(1)
        else:
            # Se non è specificata una quantità, assumiamo 1
            target_requirements[item.lower().strip()] = 1
            
    node = SnapshotNode(target_requirements)
    
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("SnapshotScript").info("Script terminato con successo.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
