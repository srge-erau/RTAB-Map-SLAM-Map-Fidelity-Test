import os # Import os for checking file existence and creating dummy files
import open3d as o3d # type: ignore
import numpy as np # type: ignore
import matplotlib.pyplot as plt # type: ignore
from scipy.spatial.transform import Rotation as R # type: ignore # For quaternion to rotation matrix conversion


def load_trajectory_from_txt(filepath, skip_header=True):
    """
    Loads trajectory data from a .txt file.
    Assumes each line contains: timestamp x y z qx qy qz qw id
    Optionally skips the first line (header).
    """
    timestamps = []
    positions = []
    orientations_quat = [] # Store quaternions (x, y, z, w)
    ids = []

    with open(filepath, 'r') as f:
        if skip_header:
            next(f) # Skips the first line (header)

        for line in f:
            parts = list(map(float, line.strip().split()))
            if len(parts) == 9: # timestamp x y z qx qy qz qw id
                timestamps.append(parts[0])
                positions.append(parts[1:4]) # x, y, z
                # Open3D expects quaternions as (x, y, z, w) for get_rotation_matrix_from_quaternion
                # Our file is qx, qy, qz, qw, which matches
                orientations_quat.append(parts[4:8]) # qx, qy, qz, qw
                ids.append(int(parts[8]))
            else:
                print(f"Warning: Unexpected line format in trajectory file (expected 9 columns, found {len(parts)}): {line.strip()}")
    
    return {
        'timestamps': np.array(timestamps),
        'positions': np.array(positions),
        'orientations_quat': np.array(orientations_quat),
        'ids': np.array(ids)
    }

def combine_and_visualize_3d(ply_file_path, trajectory_file_path):
    """
    Loads a PLY point cloud and a trajectory (with quaternion orientation),
    then visualizes them together with coordinate frames.
    """
    print(f"Loading point cloud from: {ply_file_path}")
    try:
        pcd = o3d.io.read_point_cloud(ply_file_path)
    except Exception as e:
        print(f"Error loading PLY file: {e}")
        print("Please ensure the PLY file is valid and readable.")
        return

    if not pcd.has_points():
        print("Warning: Loaded PLY file contains no points.")
        pcd = o3d.geometry.PointCloud() # Create an empty one to avoid errors later


    print(f"Loading trajectory from: {trajectory_file_path}")
    # Pass skip_header=True to load_trajectory_from_txt
    trajectory_data = load_trajectory_from_txt(trajectory_file_path, skip_header=True)

    if trajectory_data['positions'].size == 0:
        print("Warning: No trajectory data loaded (after skipping header).")
        o3d.visualization.draw_geometries([pcd])
        return

    trajectory_positions = trajectory_data['positions']
    trajectory_quaternions = trajectory_data['orientations_quat']

    # Create a LineSet object for the trajectory path
    points = trajectory_positions
    lines = []
    for i in range(len(points) - 1):
        lines.append([i, i + 1])

    colors = [[0, 0, 1] for _ in range(len(lines))] # Blue color for the path

    trajectory_line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    trajectory_line_set.colors = o3d.utility.Vector3dVector(colors)

    # Create coordinate frames (axes) at each pose
    # We'll sample these to avoid clutter for very long trajectories
    trajectory_frames = []
    # Adjust this rate: smaller for more frames, larger for fewer frames
    frame_sample_rate = 50 # Show a frame every 50 poses
    axis_size = 0.5 # Adjust the size of the axes

    for i, (position, quaternion) in enumerate(zip(trajectory_positions, trajectory_quaternions)):
        if i % frame_sample_rate == 0:
            # Create a coordinate frame at the origin
            mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_size)

            # Create a 4x4 transformation matrix from position and quaternion
            # Ensure quaternion order is (x, y, z, w) for scipy's Rotation
            # And Open3D's get_rotation_matrix_from_quaternion expects (x,y,z,w)
            R_quat = R.from_quat(quaternion) # Scipy handles (x,y,z,w)
            rotation_matrix = R_quat.as_matrix() # Convert to 3x3 rotation matrix

            # Build the 4x4 transformation matrix
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = position

            # Apply the transformation to the coordinate frame
            mesh_frame.transform(transform_matrix)
            trajectory_frames.append(mesh_frame)

    # Combine all geometries for visualization
    geometries_to_draw = [pcd, trajectory_line_set] + trajectory_frames

    print("Displaying 3D visualization...")
    o3d.visualization.draw_geometries(geometries_to_draw)
    print("Visualization complete.")

if __name__ == "__main__":
    # --- Configuration ---
    # Make sure these files exist in the same directory as the script,
    # or provide their full paths.
    PLY_FILE = "/home/srge/.ros/tests/Go2_small_rock_t4/refine_output/filtered_Go2_small_rock_t4.ply"
    TXT_FILE = "/home/srge/.ros/tests/Go2_small_rock_t4/poses_odom.txt"
    print(f"Using PLY file: {PLY_FILE}")
    print(f"Using trajectory file: {TXT_FILE}")

    '''
    # --- Create dummy files for testing if they don't exist ---
    # You can comment this out once you use your actual files
    
    if not os.path.exists(PLY_FILE):
        print(f"Creating dummy {PLY_FILE} for testing...")
        # Create a simple box point cloud
        points_box = np.array([
            [0,0,0], [1,0,0], [0,1,0], [0,0,1],
            [1,1,0], [1,0,1], [0,1,1], [1,1,1],
            [0.5,0.5,0.5] # Add a center point
        ])
        pcd_dummy = o3d.geometry.PointCloud()
        pcd_dummy.points = o3d.utility.Vector3dVector(points_box)
        o3d.io.write_point_cloud(PLY_FILE, pcd_dummy)

    if not os.path.exists(TXT_FILE):
        print(f"Creating dummy {TXT_FILE} for testing...")
        # Dummy trajectory with header: moving along X axis, facing forward
        dummy_trajectory_lines = []
        dummy_trajectory_lines.append("timestamp x y z qx qy qz qw id") # Header line
        for i in range(10):
            timestamp = i * 0.1
            x = i * 0.2
            y = 0.0
            z = 0.0
            # Quaternion for no rotation (identity): qx=0, qy=0, qz=0, qw=1
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0 # No rotation
            _id = i
            dummy_trajectory_lines.append(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw} {_id}")

        # Add a few points with rotation to demonstrate frames
        # Rotate 90 degrees around Z axis (facing along Y)
        r_z_90 = R.from_euler('z', 90, degrees=True)
        q_z_90 = r_z_90.as_quat() # (x, y, z, w)
        for i in range(10, 15):
            timestamp = i * 0.1
            x = 2.0 # Keep x constant
            y = (i - 10) * 0.2
            z = 0.0
            qx, qy, qz, qw = q_z_90[0], q_z_90[1], q_z_90[2], q_z_90[3]
            _id = i
            dummy_trajectory_lines.append(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw} {_id}")
            
        with open(TXT_FILE, 'w') as f:
            f.write("\n".join(dummy_trajectory_lines))
    # --- End of dummy file creation ---
    '''

    # --- Run the visualization ---
    combine_and_visualize_3d(PLY_FILE, TXT_FILE)
    

    # --- Optional: Visualize trajectory in 2D (e.g., top-down view) using Matplotlib ---
    
    trajectory_data_2d = load_trajectory_from_txt(TXT_FILE, skip_header=True)['positions']
    if trajectory_data_2d.size > 0:
        plt.figure(figsize=(8, 6))
        plt.plot(trajectory_data_2d[:, 0], trajectory_data_2d[:, 1], 'b-o', markersize=3, label='Trajectory (X-Y)')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('2D Trajectory Plot')
        plt.grid(True)
        plt.axis('equal') # Ensures X and Y axes have same scaling
        plt.legend()
        plt.show()