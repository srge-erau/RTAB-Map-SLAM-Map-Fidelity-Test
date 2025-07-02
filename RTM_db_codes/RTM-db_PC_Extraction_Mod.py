import sqlite3
import zlib
import numpy as np
import open3d as o3d

# RTAB-Map database
db_path = "/home/srge/.ros/tests/Go2_small_rock_t4/Go2_small_rock_t4.db"
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

cursor.execute("SELECT id, depth FROM Data WHERE depth IS NOT NULL")
rows = cursor.fetchall()
print(f"Found {len(rows)} entries with depth.")

# Common resolutions and intrinsics (RealSense D435i defaults)
intrinsics_map = {
    (480, 640): (617.0, 617.0, 320.0, 240.0),
    (360, 640): (617.0, 617.0, 320.0, 180.0),
    (240, 320): (285.0, 285.0, 160.0, 120.0),
    (720, 1280): (910.0, 910.0, 640.0, 360.0),  # HD guess
    # Add more known combos here if needed
}

def guess_resolution(n_pixels):
    for w in range(100, 2000):
        if n_pixels % w == 0:
            h = n_pixels // w
            if 100 <= h <= 1500:
                yield (h, w)

for node_id, depth_blob in rows:
    print(f"\nProcessing node ID: {node_id}")

    try:
        depth_raw = zlib.decompress(depth_blob)
    except zlib.error:
        depth_raw = depth_blob  # not compressed

    depth_array = np.frombuffer(depth_raw, dtype=np.uint16)
    total_pixels = depth_array.size

    # Guess shape
    shape_found = False
    for (h, w) in guess_resolution(total_pixels):
        if (h, w) in intrinsics_map:
            try:
                depth_img = depth_array.reshape((h, w))
                fx, fy, cx, cy = intrinsics_map[(h, w)]
                shape_found = True
                break
            except ValueError:
                continue

    if not shape_found:
        print(f"⚠️ Could not determine valid shape for node {node_id} ({total_pixels} pixels). Skipping.")
        continue

    print(f"Detected shape: {depth_img.shape}, using intrinsics fx={fx}, fy={fy}, cx={cx}, cy={cy}")

    # Open3D conversion
    o3d_depth = o3d.geometry.Image(depth_img)
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(width=w, height=h, fx=fx, fy=fy, cx=cx, cy=cy)

    pcd = o3d.geometry.PointCloud.create_from_depth_image(
        o3d_depth,
        intrinsic,
        project_valid_depth_only=True,
        depth_scale=1000.0,
        depth_trunc=4.0,
    )

    if len(pcd.points) == 0:
        print(f"⚠️ No valid points for node {node_id}. Skipping.")
        continue

    o3d.visualization.draw_geometries([pcd])
    o3d.io.write_point_cloud(f"node_{node_id}_reconstructed.pcd", pcd)
    print(f"✅ Saved node_{node_id}_reconstructed.pcd")

conn.close()
