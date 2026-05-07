# OPEN3D_VINS

OPEN3D_VINS is a VINS-Fusion based visual-inertial localization and offline dense mapping project for UAV mapping workflows. The repository combines a GPU-accelerated VINS-Fusion fork, manual loop/relocalization changes, RealSense-oriented configuration, and Open3D/PCL scripts for generating PCD maps from rosbag data and optimized VINS trajectories.

The current target platform is Jetson Orin NX with Ubuntu 20.04, ROS Noetic, JetPack 5.1.1, and a RealSense D435i/D435i-like RGB-D camera with IMU.

## Main Features

- GPU-accelerated VINS-Fusion frontend for stereo/RGB-D + IMU localization.
- Manual prior loop/relocalization mode in `loop_fusion`.
- `/vins_fusion/force_keyframe` service for forcing keyframe publication when the UAV is stationary.
- Offline dense map generation from rosbag depth frames and VINS trajectories.
- Open3D TSDF based PCD map generation with height coloring and filtering.
- PCL-style depth stitching script retained as an alternative mapping path.
- QGC/ground-station oriented coordinate design using `map -> odom_vins -> base_link`.

## Repository Layout

```text
.
├── vins-fusion-gpu/
│   ├── camera_models/        # camera model package
│   ├── vins_estimator/       # VINS estimator node and launch files
│   ├── loop_fusion/          # loop closure and manual relocalization changes
│   ├── config/FS-J200/       # RealSense/IMU configuration examples
│   └── support_files/        # BRIEF vocabulary and reference files
├── scripts/
│   ├── open3d_tsdf_from_bag.py
│   ├── open3d_tsdf_geometry_from_bag.py
│   └── pcl_depth_stitch_from_bag.py
├── pcd_analysis/             # generated map preview images
├── map.pcd                   # example generated point cloud map
└── *.md                      # design notes and current progress notes
```

## Dependencies

Install the normal VINS-Fusion dependencies first:

- ROS Noetic
- Ceres Solver
- Eigen
- OpenCV with CUDA support, tested with OpenCV 4.x
- rosbag, cv_bridge, image_transport, std_srvs
- PCL / pcl_ros for PCD visualization

For offline mapping scripts:

```bash
python3 -m pip install --user open3d numpy scipy pyyaml
```

On Jetson/ROS systems, prefer `python3 -m pip install --user ...` and avoid mixing `sudo pip3` into the ROS Python environment.

## Build

Clone this repository into a catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/levelmoon/OPEN3D_VINS.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Before building, check the OpenCV path in:

```text
vins-fusion-gpu/vins_estimator/CMakeLists.txt
vins-fusion-gpu/loop_fusion/CMakeLists.txt
```

The GPU fork expects an OpenCV build with CUDA modules.

## Run VINS-Fusion

The included FS-J200 launch file starts `vins_node` with the RealSense-oriented config:

```bash
roslaunch vins FS-J200.launch
```

Configuration entry point:

```text
vins-fusion-gpu/config/FS-J200/FS-J200_stereo_imu_config.yaml
```

GPU-related parameters:

```yaml
use_gpu: 1
use_gpu_acc_flow: 1
```

Set `use_gpu_acc_flow: 0` if other processes need more GPU resources.

## Manual Relocalization

The loop fusion code supports three modes after loading a historical pose graph:

```text
0: auto global loop
1: manual prior loop
2: hybrid manual prior + global fallback
```

Manual prior input format:

```text
x y yaw_deg radius yaw_radius_deg
```

Example:

```text
-2.2 0 -87.777 2 45
```

The input is only a candidate search prior in the pose graph/map frame. The actual loop constraint still depends on BRIEF/BoW matching and geometric verification.

When VINS does not automatically create a new keyframe, force the estimator to publish the current frame:

```bash
rosservice call /vins_fusion/force_keyframe "{}"
```

In `loop_fusion`, press:

```text
r
```

This triggers the forced keyframe path and attempts relocalization with the latest synchronized image, keyframe points, and pose.

## Offline Dense Mapping

The recommended mapping path is:

```text
rosbag
  -> VINS-Fusion optimized trajectory
  -> Open3D TSDF fusion
  -> map.pcd / height-colored PCD
```

Example command:

```bash
python3 scripts/open3d_tsdf_from_bag.py \
  --bag mapping.bag \
  --traj vins_loop_optimized.txt \
  --out map_height.pcd \
  --depth-topic /camera/aligned_depth_to_color/image_raw \
  --camera-info-topic /camera/color/camera_info \
  --color-mode height \
  --height-axis z
```

Expected trajectory format:

```text
timestamp,px,py,pz,qw,qx,qy,qz,vx,vy,vz
```

The script supports depth filtering, TSDF integration, voxel downsampling, statistical/radius outlier filtering, and height-based RGB coloring.

Visualize a generated PCD in RViz:

```bash
rosrun pcl_ros pcd_to_pointcloud map_height.pcd 1 _frame_id:=map
```

RViz settings:

```text
Fixed Frame: map
PointCloud2 Topic: /cloud_pcd
Color Transformer: RGB8 / RGB
```

## Coordinate Design

The recommended online coordinate tree is:

```text
map
 |
 | T_map_odom, updated by loop/relocalization/manual prior
 |
odom_vins
 |
 | T_odom_base, continuous VINS output
 |
base_link
```

Flight control should use continuous `odom_vins -> base_link` odometry. Loop closure or relocalization should update `map -> odom_vins` for ground-station display, QGC integration, or task-level planning, rather than directly injecting discontinuous corrected poses into the flight-control loop.

## Notes

- `map.pcd` and `pcd_analysis/` are example outputs for map inspection.
- `scripts/pcl_depth_stitch_from_bag.py` is kept as an alternative PCL-style depth stitching implementation.
- The original VINS-Fusion GPU README is preserved at `vins-fusion-gpu/READEME.md`.
- Additional implementation notes are kept in the Chinese Markdown files at the repository root.

## License

The upstream VINS-Fusion GPU code includes GPLv3 licensing information in `vins-fusion-gpu/LICENCE`. Check upstream dependency licenses before redistributing binaries or modified releases.
