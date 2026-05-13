import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import os

# =================================================================
# CALIBRAZIONE CAMERA - 13 MAGGIO 2026
# AprilTag ID 0 posizionato al CENTRO del tavolo (0,0,0)
# =================================================================

# 1. Matrice Camera (Optical Frame) -> Tag (FORNITA DA UTENTE)
T_link_tag = np.array([
    [ 0.840, -0.542, -0.027,  0.045],
    [-0.184, -0.238, -0.954,  0.164],
    [ 0.510,  0.806, -0.300,  1.590],
    [ 0.000,  0.000,  0.000,  1.000]
])

# 2. Trasformata manuale: Tavolo -> Tag
T_table_tag = np.eye(4)
# L'AprilTag ora risulta allineato agli assi del tavolo, non c'è bisogno di ruotarlo di 180°
# T_table_tag[:3, :3] = Rotation.from_euler('z', np.pi).as_matrix()

# 3. Calcolo Posa Camera rispetto al Tavolo
# T_table_link = T_table_tag @ inv(T_link_tag)
T_tag_link = np.linalg.inv(T_link_tag)
T_table_link = T_table_tag @ T_tag_link

pos = T_table_link[:3, 3]
euler = Rotation.from_matrix(T_table_link[:3, :3]).as_euler('xyz')

# 5. Aggiornamento automatico robot_poses.yaml
pkg_path = os.path.join(os.getcwd(), 'src/franka_bimanual_config/config/robot_poses.yaml')

if os.path.exists(pkg_path):
    with open(pkg_path, 'r') as f:
        data = yaml.safe_load(f)
    
    data['camera']['x'] = float(pos[0])
    data['camera']['y'] = float(pos[1])
    data['camera']['z'] = float(pos[2])
    data['camera']['roll'] = float(euler[0])
    data['camera']['pitch'] = float(euler[1])
    data['camera']['yaw'] = float(euler[2])
    
    with open(pkg_path, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False)
    
    print("\n" + "="*50)
    print("CALIBRAZIONE SALVATA CON SUCCESSO!")
    print(f"File: {pkg_path}")
    print(f"Posa finale Camera (frame table):")
    print(f"  x: {pos[0]:.4f}")
    print(f"  y: {pos[1]:.4f}")
    print(f"  z: {pos[2]:.4f}")
    print(f"  roll: {euler[0]:.4f}")
    print(f"  pitch: {euler[1]:.4f}")
    print(f"  yaw: {euler[2]:.4f}")
    print("="*50 + "\n")
else:
    print(f"ERRORE: File {pkg_path} non trovato.")
