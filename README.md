# RTAB-Map-SLAM-Map-Fidelity-Test

**Note**: In order to access these codes, .db files from RTAB-Map must be converted to .ply (Point Cloud) and .txt (Trajectory/ Camera Odometry) files.

RTM_Vis_codes: Visualization codes for PC access from .ply and .txt files. Traditional: Without Spatial Index, Refined_Vis: With STRtree Spatial Index

RTM_db_codes: Allows access to read tables within database (.db) files. Reads available headers within published topics. Useful for retrieving required odometry and point cloud info for post-processing
