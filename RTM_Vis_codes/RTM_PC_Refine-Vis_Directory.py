import open3d as o3d # type: ignore
import numpy as np # type: ignore
from scipy.spatial.transform import Rotation as R # type: ignore # For quaternion to rotation matrix conversion
import os
from shapely.geometry import Polygon, MultiPolygon, Point, LineString # For 2D point-in-polygon test # type: ignore
from shapely.strtree import STRtree # For spatial indexing of polygons # type: ignore
import matplotlib.pyplot as plt # For optional 2D plot # type: ignore

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

def filter_point_cloud_by_trajectory_loop_2d(pcd, trajectory_positions_3d):
    """
    Filters a 3D point cloud, keeping only points whose 2D (X-Y) projection
    falls within the 2D polygon formed by the trajectory loop.

    Args:
        pcd (open3d.geometry.PointCloud): The input point cloud.
        trajectory_positions_3d (np.ndarray): Nx3 array of trajectory (x,y,z) positions.

    Returns:
        open3d.geometry.PointCloud: A new point cloud containing only the filtered points.
    """
    if not pcd.has_points() or trajectory_positions_3d.shape[0] < 3:
        print("Not enough points in PCD or trajectory to form a loop for filtering.")
        return o3d.geometry.PointCloud() # Return an empty point cloud

    # 1. Project trajectory to 2D (X-Y plane)
    trajectory_2d = trajectory_positions_3d[:, :2] # Take only X and Y

    # 2. Form a Shapely Polygon from the projected trajectory
    # Ensure the trajectory forms a closed loop or sufficiently large boundary
    # For a simple trajectory, using all points might create a complex self-intersecting polygon.
    # Consider simplifying the trajectory if it's very dense, or if it doesn't close perfectly.
    # For a general loop, we assume the first and last points are close.
    # We can try to close the loop explicitly if not already closed
    if not np.allclose(trajectory_2d[0], trajectory_2d[-1], atol=0.1): # Check if start and end are close (adjust tolerance)
        print("Warning: Trajectory does not appear to form a closed loop. Attempting to close it.")
        # Add the first point to the end to close the polygon
        closed_trajectory_2d = np.vstack((trajectory_2d, trajectory_2d[0]))
    else:
        closed_trajectory_2d = trajectory_2d
    '''''''''
    # Create the Shapely Polygon. If the trajectory is self-intersecting,
    # Polygon creation might yield a MultiPolygon or an invalid polygon.
    # For simplicity, we'll try to create a single Polygon.
    try:
        # Simplification can help if the trajectory is very dense or noisy,
        # otherwise, a self-intersecting polygon can be problematic for contains().
        epsilon = 0.1 # Adjust this value as needed for simplification
        simplified_trajectory_2d = LineString(closed_trajectory_2d).simplify(epsilon)
        poly = Polygon(simplified_trajectory_2d)
        
        # Directly use points, Shapely can handle some self-intersections
        # but for robustness, ensure your trajectory is reasonable.
        poly = Polygon(closed_trajectory_2d)
        if not poly.is_valid:
            print("Warning: Generated 2D polygon is invalid. Attempting to buffer to fix.")
            # A common trick to fix invalid polygons is to buffer by a small amount and then by negative that amount
            poly = poly.buffer(0.001).buffer(-0.001)
            if not poly.is_valid:
                print("Error: Polygon remains invalid after buffer fix. Cannot filter points.")
                return o3d.geometry.PointCloud()

    except Exception as e:
        print(f"Error creating Shapely Polygon from trajectory: {e}")
        print("Returning empty filtered point cloud.")
        return o3d.geometry.PointCloud()
    '''
# Create the Shapely geometry. This might be a Polygon or a MultiPolygon.
    try:
        # Use LineString to handle potential complexities before forming a Polygon
        # This helps in simplifying potential self-intersections
        line = LineString(closed_trajectory_2d)
        
        # Buffer operation can "clean" invalid geometries or merge close parts,
        # often resulting in a MultiPolygon if parts are disconnected.
        # A small positive buffer, then a small negative buffer can simplify things.
        # Adjust buffer_amount based on the scale of your trajectory.
        buffer_amount = 1 # A small buffer to fix geometry, adjust if needed
        buffered_line = line.buffer(buffer_amount, join_style='round')

        # Now try to convert the buffered result to a polygon.
        # This might still yield a MultiPolygon.
        poly_or_multipoly = buffered_line.buffer(-buffer_amount, join_style='round')

        if not poly_or_multipoly.is_valid:
            print("Warning: Generated 2D polygon/multipolygon is invalid even after buffer. Skipping filter.")
            return o3d.geometry.PointCloud()

    except Exception as e:
        print(f"Error creating Shapely Polygon/MultiPolygon from trajectory: {e}")
        print("Returning empty filtered point cloud.")
        return o3d.geometry.PointCloud()

# Determine the geometries to check against (can be one Polygon or many)
    polygons_to_check = []
    if isinstance(poly_or_multipoly, Polygon):
        polygons_to_check.append(poly_or_multipoly)
        print(f"2D polygon from trajectory has {len(poly_or_multipoly.exterior.coords)} vertices.")
    elif isinstance(poly_or_multipoly, MultiPolygon):
        polygons_to_check.extend(poly_or_multipoly.geoms)
        print(f"2D MultiPolygon from trajectory has {len(polygons_to_check)} parts.")
    else:
        print(f"Warning: Unexpected geometry type ({type(poly_or_multipoly)}). Skipping filter.")
        return o3d.geometry.PointCloud()
    
    if not polygons_to_check:
        print("No valid polygons to filter against. Returning empty point cloud.")
        return o3d.geometry.PointCloud()

    '''''print(f"2D polygon from trajectory has {len(poly.exterior.coords)} vertices.")
    '''
# Create a spatial index (STRtree) of the polygons
    print(f"Building STRtree for {len(polygons_to_check)} polygons...")
    tree = STRtree(polygons_to_check)
    print("STRtree built.")

    # 3. Get all points from the original point cloud
    points_3d = np.asarray(pcd.points)
    
    # 4. Project point cloud points to 2D
    points_2d = points_3d[:, :2]

    # 5. Check which points are inside the 2D polygon
    # This can be slow for very large point clouds.
    # For optimization, consider using a spatial index (e.g., from RTree or directly with geopandas)
    # or batch processing with shapely.geometry.MultiPoint and poly.contains(multipoint)
    
    '''''''''
    # Simple loop for demonstration
    inside_indices = []
    for i, p_2d in enumerate(points_2d):
        if poly.contains(Point(p_2d)):
            inside_indices.append(i)
    '''

    inside_indices = []

    # --- OPTIMIZED: Use STRtree for faster lookups ---
    # Create a list of Shapely Point objects for the point cloud points
    shapely_points = [Point(p_2d) for p_2d in points_2d]

    # Query the STRtree to find potential polygon overlaps for each point's bounding box
    # The `query` method returns indices of the geometries in the tree that intersect the query geometry.
    # We use a point as the query geometry, so it finds polygons that contain that point (or its tiny bounding box).
    # This is much faster than iterating all polygons for each point.
    
    # Using `tree.geoms` directly or `tree.query` can be tricky with individual points.
    # The most direct way to leverage STRtree for point-in-polygon is often to iterate
    # through the points and query for each, or, if you have very many points,
    # consider approaches like `geopandas.sjoin` which uses spatial indexes internally
    # and is highly optimized.

    # For pure Shapely, the recommended way for many points against many polygons
    # is often to build an STRtree of the polygons and then query for each point.
    
    # This approach is still N_points * log(N_polygons) (approx)
    # The `query` method can return many potential matches if polygons overlap.
    
    # Let's stick with iterating points and querying the tree.
    # The `query` method returns the *indices* of the geometries in the tree that intersect
    # the given bounding box or geometry.
    
    for i, point_shapely in enumerate(shapely_points):
        # Query the tree for polygons whose bounding boxes intersect with the point's bounding box
        # This returns a list of indices of `polygons_to_check`
        possible_matches_indices = tree.query(point_shapely, predicate='intersects') # Use 'intersects' for quick bbox check
        
        # Now, for the exact check, iterate only through the *possible* matches
        is_inside = False
        for poly_idx in possible_matches_indices:
            if polygons_to_check[poly_idx].contains(point_shapely):
                inside_indices.append(i)
                is_inside = True
                break # Point is inside at least one polygon, no need to check other matches
        
    print(f"Found {len(inside_indices)} points inside the trajectory loop (2D projection).")
    # --- END OPTIMIZED ---

    '''''''''
    for i, p_2d in enumerate(points_2d):
        point_shapely = Point(p_2d)
        for poly in polygons_to_check:
            if poly.contains(point_shapely):
                inside_indices.append(i)
                break # Point is inside at least one polygon, no need to check others
    print(f"Found {len(inside_indices)} points inside the trajectory loop (2D projection).")
    '''
    # 6. Create a new point cloud with only the filtered points
    filtered_pcd = o3d.geometry.PointCloud()
    if inside_indices:
        filtered_pcd.points = o3d.utility.Vector3dVector(points_3d[inside_indices])
        if pcd.has_colors():
            filtered_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[inside_indices])
        if pcd.has_normals():
            filtered_pcd.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals)[inside_indices])
    
    return filtered_pcd

def combine_and_visualize_3d(ply_filename, txt_filename, ply_dir, txt_dir, output_dir, filter_pcd=False):
    """
    Loads a PLY point cloud and a trajectory (with quaternion orientation),
    then visualizes them together with coordinate frames.
    Optionally filters the point cloud to only show points within the trajectory loop.
    Allows specifying input and output directories.
    """
   # Construct full paths for input files
    ply_dir = "/home/srge/Documents/Karthik/Tests/Pretrials/Pretrials_R0.01"
    ply_filename = "Pretrials_R0.01.ply"
    txt_dir = "/home/srge/Documents/Karthik/Tests/Pretrials/Pretrials_R0.01"
    txt_filename = "Pretrials_R0.01.txt"
    ply_file_path = os.path.join(ply_dir, ply_filename)
    trajectory_file_path = os.path.join(txt_dir, txt_filename)

    # Ensure output directory exists
    output_dir = "/home/srge/Documents/Karthik/Tests/Pretrials_Output"
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output files will be saved to: {output_dir}")

    print(f"Loading point cloud from: {ply_file_path}")
    try:
        pcd = o3d.io.read_point_cloud(ply_file_path)
    except Exception as e:
        print(f"Error loading PLY file: {e}")
        print("Please ensure the PLY file is valid and readable.")
        return

    if not pcd.has_points():
        print("Warning: Loaded PLY file contains no points.")
        pcd = o3d.geometry.PointCloud()


    print(f"Loading trajectory from: {trajectory_file_path}")
    trajectory_data = load_trajectory_from_txt(trajectory_file_path, skip_header=True)

    if trajectory_data['positions'].size == 0:
        print("Warning: No trajectory data loaded (after skipping header).")
        o3d.visualization.draw_geometries([pcd])
        return

    trajectory_positions = trajectory_data['positions']
    trajectory_quaternions = trajectory_data['orientations_quat']

    # --- Point Cloud Filtering ---
    if filter_pcd:
        print("Filtering point cloud based on trajectory loop...")
        original_pcd_num_points = len(pcd.points)
        pcd = filter_point_cloud_by_trajectory_loop_2d(pcd, trajectory_positions)
        print(f"Reduced point cloud from {original_pcd_num_points} to {len(pcd.points)} points.")
        # If filtered_pcd is empty, pcd will be empty here, and the visualization will still work.

        # Save the filtered point cloud to the output directory
        filtered_ply_output_path = os.path.join(output_dir, f"filtered_{ply_filename}")
        o3d.io.write_point_cloud(filtered_ply_output_path, pcd)
        print(f"Filtered point cloud saved to: {filtered_ply_output_path}")

    # --- End Point Cloud Filtering ---

    # Create a LineSet object for the trajectory path
    points_line = trajectory_positions
    lines = []
    for i in range(len(points_line) - 1):
        lines.append([i, i + 1])

    colors = [[0, 0, 1] for _ in range(len(lines))] # Blue color for the path

    trajectory_line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points_line),
        lines=o3d.utility.Vector2iVector(lines),
    )
    trajectory_line_set.colors = o3d.utility.Vector3dVector(colors)
    
    # Create coordinate frames (axes) at each pose
    # trajectory_frames = []

    frame_sample_rate = 10 # Show a frame every 10 poses (adjust as needed)
    axis_size = 0.3 # Adjust the size of the axes
    
    for i, (position, quaternion) in enumerate(zip(trajectory_positions, trajectory_quaternions)):
        if i % frame_sample_rate == 0:
            mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=axis_size)
            R_quat = R.from_quat(quaternion)
            rotation_matrix = R_quat.as_matrix()
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = position
            mesh_frame.transform(transform_matrix)
            # trajectory_frames.append(mesh_frame)

    # Combine all geometries for visualization
    geometries_to_draw = [pcd, trajectory_line_set] #+ trajectory_frames

    print("Displaying 3D visualization...")
    o3d.visualization.draw_geometries(geometries_to_draw)
    print("Visualization complete.")

if __name__ == "__main__":
    # --- Configuration ---
    PLY_FILE = "Pretrials_R0.1.ply"
    TXT_FILE = "/home/srge/Documents/Karthik/Tests/Pretrials/Pretrials_R0.1/Pretrials_R0.1.txt"

    # Define your input and output directories here
    INPUT_PLY_DIR = "/home/srge/Documents/Karthik/Tests/Pretrials/Pretrials_R0.1"
    INPUT_TXT_DIR = "/home/srge/Documents/Karthik/Tests/Pretrials/Pretrials_R0.1"
    OUTPUT_DIR = "/home/srge/Documents/Karthik/Tests/Pretrials_Output"  


    '''''
    # --- Create dummy files for testing if they don't exist ---
    # This part helps in running the script without manually creating files.
    # You can comment it out if you always provide real data.
    os.makedirs(INPUT_PLY_DIR, exist_ok=True)
    os.makedirs(INPUT_TXT_DIR, exist_ok=True)

    dummy_ply_path = os.path.join(INPUT_PLY_DIR, PLY_FILENAME)
    dummy_txt_path = os.path.join(INPUT_TXT_DIR, TXT_FILENAME)

    if not os.path.exists(dummy_ply_path):
        print(f"Creating dummy {dummy_ply_path} for testing...")
        np.random.seed(42)
        points_count = 500000 # Increased to 500,000 points to really show the speedup
        points_box = np.random.rand(points_count, 3) * [4.0, 4.0, 1.0] - [2.0, 2.0, 0.5]
        
        pcd_dummy = o3d.geometry.PointCloud()
        pcd_dummy.points = o3d.utility.Vector3dVector(points_box)
        colors_dummy = np.random.rand(points_count, 3)
        pcd_dummy.colors = o3d.utility.Vector3dVector(colors_dummy)

        o3d.io.write_point_cloud(dummy_ply_path, pcd_dummy)

    if not os.path.exists(dummy_txt_path):
        print(f"Creating dummy {dummy_txt_path} for testing...")
        dummy_trajectory_lines = []
        dummy_trajectory_lines.append("timestamp x y z qx qy qz qw id")
        
        # A more complex trajectory to potentially generate many polygon parts
        # This aims for a "scribble" like path or figure-eight that results in many components.
        num_spiral_points = 500 # More points for a complex path
        for i in range(num_spiral_points):
            angle = i * 0.1
            radius = 0.5 + i * 0.002 # Growing spiral
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = 0.0 # Keep it flat for 2D projection
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0 # No rotation
            dummy_trajectory_lines.append(f"{len(dummy_trajectory_lines)*0.01:.3f} {x:.3f} {y:.3f} {z:.3f} {qx:.3f} {qy:.3f} {qz:.3f} {qw:.3f} {len(dummy_trajectory_lines)-1}")
        
        # Add a few disconnected loops to really force a MultiPolygon with many parts
        for j in range(3): # 3 additional small disconnected loops
            center_x = 1.0 + j * 0.5
            center_y = 1.0 + j * 0.5
            for i in range(20):
                angle = i * 2 * np.pi / 20
                x = center_x + np.cos(angle) * 0.1
                y = center_y + np.sin(angle) * 0.1
                z = 0.0
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
                dummy_trajectory_lines.append(f"{len(dummy_trajectory_lines)*0.01:.3f} {x:.3f} {y:.3f} {z:.3f} {qx:.3f} {qy:.3f} {qz:.3f} {qw:.3f} {len(dummy_trajectory_lines)-1}")

        with open(dummy_txt_path, 'w') as f:
            f.write("\n".join(dummy_trajectory_lines))
    # --- End of dummy file creation ---
    '''
    
    # --- Run the visualization ---
    # Set filter_pcd=True to filter the point cloud
    combine_and_visualize_3d(
        PLY_FILE, 
        TXT_FILE, 
        INPUT_PLY_DIR, 
        INPUT_TXT_DIR, 
        OUTPUT_DIR, 
        filter_pcd=True
    ) 
    # combine_and_visualize_3d(PLY_FILE, TXT_FILE, filter_pcd=False) # To see without filtering

    # --- Optional: Visualize trajectory in 2D (e.g., top-down view) using Matplotlib ---
    # This part remains the same for quick 2D check
    trajectory_positions_2d_plot = load_trajectory_from_txt(TXT_FILE, skip_header=True)['positions'][:, :2]
    if trajectory_positions_2d_plot.size > 0:
        plt.figure(figsize=(8, 6))
        plt.plot(trajectory_positions_2d_plot[:, 0], trajectory_positions_2d_plot[:, 1], 'b-o', markersize=3, label='Trajectory (X-Y)')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('2D Trajectory Plot')
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        plt.show()