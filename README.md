# RTAB-Map SLAM Map-Fidelity Tools

Research scripts for inspecting RTAB-Map SQLite databases, reconstructing point clouds, visualizing robot trajectories, and filtering point-cloud geometry for map-fidelity studies.

## Tool groups

| Directory | Purpose |
| --- | --- |
| `RTM_db_codes/` | Inspect RTAB-Map database tables/columns and prototype point-cloud extraction |
| `RTM_Vis_codes/` | Combine `.ply` point clouds with text trajectories and compare traditional versus STRtree-accelerated filtering |

## Setup

```bash
git clone https://github.com/srge-erau/RTAB-Map-SLAM-Map-Fidelity-Test.git
cd RTAB-Map-SLAM-Map-Fidelity-Test
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

SQLite support is included with most Python installations. Open3D requires a supported desktop platform for interactive visualization.

## Input data

The visualization scripts expect:

- a `.ply` point cloud exported or reconstructed from RTAB-Map; and
- a whitespace-delimited trajectory text file with a header followed by
  `timestamp x y z qx qy qz qw id`.

The database scripts operate directly on RTAB-Map `.db` files. Make a copy of valuable mapping data before experimenting.

## Usage

The current scripts contain experiment-specific paths in their configuration blocks. Edit `db_path`, `PLY_FILE`, `TXT_FILE`, and directory constants near the top or bottom of the selected script, then run it. For example:

```bash
# List tables in an RTAB-Map database after setting db_path
python RTM_db_codes/RTM-db_Table_Find.py

# Display a point cloud with its trajectory after setting PLY_FILE/TXT_FILE
python RTM_Vis_codes/RTM_PC_Trajectory_Visualization.py
```

For spatial filtering, compare:

```bash
python RTM_Vis_codes/RTM_PC_Refined_Traditional-Vis.py
python RTM_Vis_codes/RTM_PC_Refined_Visualization.py
```

The latter uses Shapely's `STRtree` spatial index. `RTM_PC_Refine-Vis_Directory.py` applies the workflow to directories of matching point-cloud and trajectory files.

## Caveats

- File paths are not yet exposed as command-line options.
- RTAB-Map database schemas can vary across versions and recording configurations.
- Validate quaternion order, coordinate frames, scale, and trajectory timestamps before interpreting map error.
- The extraction scripts are prototypes and may require adaptation to the compression/serialization used in a particular database.

## Maintainer

[Space Robotics and Generative Estimation (SRGE) Lab](https://github.com/srge-erau), Embry-Riddle Aeronautical University.
