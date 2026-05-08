# VINS-Surfel-Mapper

VINS-Surfel-Mapper is a UAV visual-inertial localization and offline dense mapping workspace. It combines a GPU-oriented VINS-Fusion fork, FS-J200/D435i configuration, Open3D mapping scripts, and a DenseSurfelMapping based offline depth-only mapper.

The current validated mapping path is:

```text
mapping_mavros.bag
  + vio.txt or vio_loop.txt
  + /camera/depth/image_rect_raw
  + /camera/depth/camera_info
  -> offline_surfel_from_bag
  -> height-colored PCD
```

This path does not run VINS during mapping. The Surfel mapper uses only raw depth images and depth camera intrinsics as image input; it does not read RGB/color images and does not use `aligned_depth_to_color`. The grayscale image required by the surfel superpixel stage is generated from the depth image itself.

## Repository Layout

```text
.
|-- DenseSurfelMapping/
|   `-- DenseSurfelMapping/
|       `-- surfel_fusion/
|           |-- src/offline_surfel_from_bag.cpp
|           |-- src/surfel_map.cpp
|           |-- src/surfel_map.h
|           |-- CMakeLists.txt
|           `-- package.xml
|-- vins-fusion-gpu/
|   |-- vins_estimator/
|   |-- loop_fusion/
|   |-- camera_models/
|   `-- config/FS-J200/
|       |-- FS-J200_stereo_imu_config.yaml
|       |-- left.yaml
|       `-- right.yaml
|-- scripts/
|   |-- open3d_tsdf_from_bag.py
|   |-- open3d_tsdf_geometry_from_bag.py
|   `-- pcl_depth_stitch_from_bag.py
|-- pcd_analysis/
`-- README.md
```

## Platform

The tested target is:

- Jetson Orin NX
- Ubuntu 20.04
- ROS Noetic
- JetPack 5.1.1
- Intel RealSense D435i
- PX4/MAVROS IMU topic: `/mavros/imu/data`

## Dependencies

Install normal VINS-Fusion and ROS mapping dependencies:

- ROS Noetic
- Ceres Solver
- Eigen
- OpenCV
- PCL and `pcl_ros`
- `rosbag`
- `cv_bridge`
- `sensor_msgs`
- `nav_msgs`

For Open3D scripts:

```bash
python3 -m pip install --user open3d numpy scipy pyyaml
```

If using depth image cleanup options in `scripts/open3d_tsdf_from_bag.py`, also install OpenCV for Python:

```bash
python3 -m pip install --user opencv-python
```

## Build

Place this repository in a catkin workspace and build:

```bash
cd ~/FS-J200
catkin_make
source devel/setup.bash
```

The offline surfel mapper target is:

```text
surfel_fusion/offline_surfel_from_bag
```

If you change `offline_surfel_from_bag.cpp`, rebuild before running:

```bash
cd ~/FS-J200
catkin_make --pkg surfel_fusion
source devel/setup.bash
```

## VINS Configuration

The active FS-J200 configuration is:

```text
vins-fusion-gpu/config/FS-J200/FS-J200_stereo_imu_config.yaml
```

Important settings:

```yaml
imu_topic: "/mavros/imu/data"
image0_topic: "/camera/infra1/image_rect_raw"
image1_topic: "/camera/infra2/image_rect_raw"
estimate_extrinsic: 0
```

The VINS trajectory files write IMU/body poses:

```text
timestamp,px,py,pz,qw,qx,qy,qz,vx,vy,vz
```

Therefore the offline depth mapper uses:

```text
T_world_depth = T_world_body * T_body_depth
```

`T_body_depth` is resolved as follows:

1. If the VINS yaml contains `body_T_depth`, use it directly.
2. Otherwise read `body_T_cam0` and `body_T_cam1`, take the stereo center, then apply the depth camera offset relative to the stereo center.

For the current D435i/PX4 setup, the validated depth offset is:

```text
T_stereoCenter_depth translation = [-0.025, 0, 0] meters
```

The resulting default `T_body_depth` from the current FS-J200 yaml is approximately:

```text
[  0.00474110   0.02964256   0.99954932   0.05966757
  -0.99875350   0.04980785   0.00326023   0.02882350
  -0.04968876  -0.99831884   0.02984176   0.04327493
   0.00000000   0.00000000   0.00000000   1.00000000 ]
```

## Offline Depth-Only Surfel Mapping

Recommended command:

```bash
rosrun surfel_fusion offline_surfel_from_bag \
  --bag mapping_mavros.bag \
  --traj vio_loop.txt \
  --out map_surfel_final_test.pcd \
  --depth-topic /camera/depth/image_rect_raw \
  --camera-info-topic /camera/depth/camera_info \
  --vins-config-yaml /home/nvidia/FS-J200/src/vins-fusion-gpu/config/FS-J200/FS-J200_stereo_imu_config.yaml \
  --height-axis z \
  --max-pose-dt 0.05 \
  --max-angular-rate 0.3 \
  --skip-first-sec 10.0 \
  --skip-last-sec 5.0 \
  --stereo-depth-tx -0.025 \
  --stereo-depth-ty 0 \
  --stereo-depth-tz 0 \
  --trajectory-keep-radius 2.5 \
  --density-voxel-size 0.20 \
  --density-min-neighbors 4 \
  --low-height-percentile 0.02 \
  --low-height-margin 0.05
```

### What The Mapper Does

- Reads raw depth from `/camera/depth/image_rect_raw`.
- Reads depth intrinsics from `/camera/depth/camera_info`.
- Uses depth as the only image input to the Surfel pipeline.
- Does not subscribe to color/RGB images.
- Does not require `/camera/aligned_depth_to_color/image_raw`.
- Reads VINS trajectory timestamps, including nanosecond timestamps.
- Interpolates VINS poses to each depth frame timestamp.
- Converts raw depth into meters.
- Builds depth-derived grayscale images for Surfel superpixels.
- Applies `T_world_body * T_body_depth`.
- Saves a height-colored PCD.

### Noise Filtering

Useful controls:

```text
--max-angular-rate 0.3
--skip-first-sec 10.0
--skip-last-sec 5.0
--trajectory-keep-radius 2.5
--density-voxel-size 0.20
--density-min-neighbors 4
--low-height-percentile 0.02
--low-height-margin 0.05
```

`--trajectory-keep-radius` keeps only points near the VIO trajectory in the XY plane. This replaces hand-written polygon crop as the main ROI method.

Manual crop options are still available for experiments:

```text
--crop-x-min / --crop-x-max
--crop-y-min / --crop-y-max
--crop-z-min / --crop-z-max
--crop-polygon "x1,y1;x2,y2;x3,y3"
```

## Open3D TSDF Mapping

The Open3D script is retained as an alternative mapping path:

```bash
python3 scripts/open3d_tsdf_from_bag.py \
  --bag mapping_mavros.bag \
  --traj vio_loop.txt \
  --out map_open3d_height.pcd \
  --depth-topic /camera/depth/image_rect_raw \
  --camera-info-topic /camera/depth/camera_info \
  --depth-trunc 3.0 \
  --color-mode height \
  --height-axis z
```

Use this path for quick TSDF experiments. Use `offline_surfel_from_bag` for the validated DenseSurfelMapping depth-only pipeline.

## Visualizing PCD Output

In RViz:

```bash
rosrun pcl_ros pcd_to_pointcloud map_surfel_final_test.pcd 1 _frame_id:=map
```

Recommended RViz settings:

```text
Fixed Frame: map
PointCloud2 Topic: /cloud_pcd
Color Transformer: RGB8 or RGB
```

## Notes

- Do not use `/camera/aligned_depth_to_color/image_raw` for the validated surfel pipeline.
- Do not require RGB frames for offline surfel mapping.
- If the map is rotated incorrectly, first verify that the trajectory is an IMU/body trajectory and that the yaml path passed to `--vins-config-yaml` matches the VINS run that produced `vio_loop.txt`.
- Generated PCD files, zip archives, and local preview images should not be committed unless they are intentionally used as small examples.

## License

The upstream VINS-Fusion GPU code includes GPLv3 licensing information in `vins-fusion-gpu/LICENCE`. Check upstream dependency licenses before redistributing binaries or modified releases.
