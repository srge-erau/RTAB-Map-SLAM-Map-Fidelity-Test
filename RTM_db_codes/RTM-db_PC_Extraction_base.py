import sqlite3
import zlib
import numpy as np
import open3d as o3d
import cv2
import struct
import matplotlib.pyplot as plt

# RTAB-Map database
db_path = "/home/srge/.ros/tests/Go2_small_rock_t4/Go2_small_rock_t4.db"
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# Get rows with depth (ignoring invalid calibration)
cursor.execute("SELECT id, depth FROM Data WHERE depth IS NOT NULL")
rows = cursor.fetchall()
print(f"Found {len(rows)} entries with depth.")

for node_id, depth_blob in rows[:]:  # limit to a few frames by adjusting [:]
    print(f"\nProcessing node ID: {node_id}")

    # Step 1: Decompress depth
    try:
        depth_raw = zlib.decompress(depth_blob)
    except zlib.error:
        print("Depth data is not compressed.")
        depth_raw = depth_blob

    # Step 2: Read and reshape depth image
    depth_img = np.frombuffer(depth_raw, dtype=np.uint16)

    print(f"Raw depth data size: {depth_img.size} pixels")

    
    plt.imshow(depth_img, cmap='gray')
    plt.title(f"Depth Image for Node {node_id}")
    plt.show()

    
    # Assuming RealSense D435i: 356x335 resolution
    try:
        depth_img = depth_img.reshape((356, 335))  # height, width
    except ValueError as e:
        print(f"Skipping node {node_id}: invalid shape - {e}")
        continue

    # Step 3: Use known RealSense D435i intrinsics
    fx, fy = 320.0, 320.0
    cx, cy = 167.5,178.0 
    print(f"Using RealSense D435i intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}")

    # Step 4: Convert depth image to Open3D format
    o3d_depth = o3d.geometry.Image(depth_img)
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(width=335, height=356, fx=fx, fy=fy, cx=cx, cy=cy)

    # Step 5: Convert to point cloud
    pcd = o3d.geometry.PointCloud.create_from_depth_image(
        o3d_depth,
        intrinsic,
        project_valid_depth_only=True,
        depth_scale=1000.0,  # RealSense D435i provides depth in mm
        depth_trunc=4.0,     # Ignore anything beyond 4 meters
    )

    if len(pcd.points) == 0:
        print(f"Warning: No valid points in point cloud for node {node_id}. Skipping.")
        continue

    # Step 6: Save and visualize
    output_path = f"node_{node_id}_reconstructed.pcd"
    o3d.io.write_point_cloud(output_path, pcd)
    print(f"Saved point cloud to {output_path}")

    o3d.visualization.draw_geometries([pcd])

conn.close()
