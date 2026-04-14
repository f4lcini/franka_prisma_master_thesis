import numpy as np
from scipy.spatial.transform import Rotation

def mock_process_point_cloud(depth_image, bbox, intrinsics, tolerance=0.05, min_pts=10):
    x1, y1, x2, y2 = bbox
    fx, fy = intrinsics['fx'], intrinsics['fy']
    cx, cy = intrinsics['cx'], intrinsics['cy']
    
    all_depths = []
    # 1. Collect all valid depths
    for vp in range(y1, y2):
        for up in range(x1, x2):
            d = float(depth_image[vp, up])
            if d > 0.001:
                all_depths.append(d)
    
    if not all_depths:
        return None, None, 0

    # 2. Median Depth
    median_depth = np.median(all_depths)
    
    # 3. Filter
    pts = []
    for vp in range(y1, y2):
        for up in range(x1, x2):
            d = float(depth_image[vp, up])
            if d > 0.001:
                z = d
                if abs(z - median_depth) <= tolerance:
                    pts.append([(up - cx) * z / fx, (vp - cy) * z / fy, z])
    
    if len(pts) < min_pts:
        return None, None, len(pts)

    pts = np.array(pts)
    centroid = pts.mean(axis=0)
    
    # 4. PCA
    cov = np.cov((pts - centroid).T)
    eigenvalues, vecs = np.linalg.eigh(cov)
    idx = np.argsort(eigenvalues)[::-1]
    vecs = vecs[:, idx]
    R = vecs.T
    if np.linalg.det(R) < 0:
        R[2, :] *= -1
    quat = Rotation.from_matrix(R).as_quat()
    
    return quat, centroid, len(pts)

# --- TEST CASE ---
if __name__ == "__main__":
    intrinsics = {'fx': 500, 'fy': 500, 'cx': 320, 'cy': 240}
    bbox = [280, 200, 360, 280] # 80x80 box

    # Create a depth image with an "object" at 1.0m and "background" at 2.0m
    depth_image = np.full((480, 640), 2.0)
    depth_image[210:270, 290:350] = 1.0 # 60x60 object at 1.0m inside the 80x80 bbox

    print("Running Perception Logic Test...")
    q, c, n = mock_process_point_cloud(depth_image, bbox, intrinsics)

    print(f"Number of points: {n}")
    print(f"Centroid: {c}")
    print(f"Median Depth expected: 1.0, Got centroid Z: {c[2] if c is not None else 'None'}")

    # Verify that background (2.0m) was filtered out
    if c is not None and abs(c[2] - 1.0) < 0.01:
        print("SUCCESS: Background filtered out correctly.")
    else:
        print("FAILURE: Centroid affected by background.")
