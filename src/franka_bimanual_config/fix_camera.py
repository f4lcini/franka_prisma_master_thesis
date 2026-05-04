import numpy as np
try:
    from scipy.spatial.transform import Rotation as R
except ImportError:
    print("Scipy not found. Please install with: pip3 install scipy")
    exit(1)

# DATI REALI DAL TUO TF2_ECHO
t_opt_tag = np.array([-0.078, 0.194, 1.686])
r_opt_tag_rpy = [2.217, -0.444, 2.800]

# 1. Matrice Tag rispetto a Camera Optical
R_opt_tag = R.from_euler('xyz', r_opt_tag_rpy).as_matrix()
T_opt_tag = np.eye(4)
T_opt_tag[:3, :3] = R_opt_tag
T_opt_tag[:3, 3] = t_opt_tag

# 2. Trasformata fissa: Camera Link -> Camera Optical
# Standard ROS: camera_link (X forward) to optical (Z forward)
R_link_opt = R.from_euler('xyz', [-np.pi/2, 0, -np.pi/2]).as_matrix()
T_link_opt = np.eye(4)
T_link_opt[:3, :3] = R_link_opt

# 3. Calcolo Camera Link rispetto al Tag (che è il centro tavolo)
# T_table_link = inv(T_link_opt * T_opt_tag)
T_link_tag = T_link_opt @ T_opt_tag
T_table_link = np.linalg.inv(T_link_tag)

pos = T_table_link[:3, 3]
euler = R.from_matrix(T_table_link[:3, :3]).as_euler('xyz')

print("\n--- COPIA QUESTO NEL FILE robot_poses.yaml ---\n")
print("camera:")
print(f"  x: {pos[0]:.4f}")
print(f"  y: {pos[1]:.4f}")
print(f"  z: {pos[2]:.4f}")
print(f"  roll: {euler[0]:.4f}")
print(f"  pitch: {euler[1]:.4f}")
print(f"  yaw: {euler[2]:.4f}")
print("\n----------------------------------------------\n")
