# 基于 VINS-Fusion + 离线稠密地图 + QGC 的无人机视觉定位与地图可视化方案

## 1. 目标

在 Jetson Orin NX、Ubuntu 20.04、ROS1、JetPack 5.1.1 环境下，实现无人机实时视觉惯性定位、回环/重定位修正、地面站高质量稠密地图可视化和任务调度。

核心原则：

- 机载端只承担实时定位、状态发布、坐标转换和飞控接口。
- 稠密地图离线构建，转换为 `PCD` 文件后由地面站加载。
- 飞控使用连续、不跳变的 VINS odometry。
- QGC/地面站使用重定位或回环修正后的 map pose。
- 回环或手动重定位只更新 `map -> odom_vins`，不直接冲击飞控控制链路。

## 2. 系统总体架构

```text
机载端 Jetson Orin NX
--------------------------------------------------
RealSense D453i/D435i/D455 + IMU
        |
        v
realsense2_camera
        |
        v
VINS-Fusion
        |
        | 连续 VIO 位姿
        v
vins_map_bridge
        |
        |------------------> MAVROS / 飞控 EKF
        |                    使用连续 odom_vins 位姿
        |
        |------------------> 地面站 / QGC
                             使用 map 坐标下的无人机位置


地面站
--------------------------------------------------
QGC
  - 任务调度
  - 航点管理
  - 无人机状态显示

独立地图可视化程序，建议 RViz / Foxglove / Potree / 自定义 WebGL
  - 加载离线 PCD 地图
  - 显示无人机 map pose
  - 显示历史轨迹、任务点、禁飞区
```

## 3. 坐标系设计

推荐使用三层坐标：

```text
map
 |
 | T_map_odom，可由手动初始化、回环、重定位更新
 |
odom_vins
 |
 | T_odom_base，VINS-Fusion 连续输出
 |
base_link
```

含义：

- `odom_vins -> base_link`：连续、低延迟、不跳变，用于飞控控制。
- `map -> odom_vins`：可被重定位或回环修正，用于地面站显示和任务调度。
- `map -> base_link`：地面站看到的无人机在离线地图中的位置。

计算关系：

```text
T_map_base = T_map_odom * T_odom_base
```

关键要求：

- 飞控不要直接使用回环优化后的跳变位姿。
- QGC/地面站可以使用回环或重定位修正后的 map pose。
- 重定位发生后，只更新 `T_map_odom`。

## 4. 离线稠密地图构建方案

当前推荐主链路：

```text
rosbag
  |
  | aligned depth + camera info
  v
VINS-Fusion 离线运行
  |
  | 回环优化轨迹
  v
PCL 风格深度点云拼接
  |
  | depth 投影 + VINS 位姿变换 + voxel grid 融合
  v
map.pcd
```

备选算法链路：

```text
rosbag
  |
  | color image + aligned depth + camera info + imu
  v
VINS-Fusion 离线运行
  |
  | 回环优化轨迹
  v
Open3D TSDF / ScalableTSDFVolume
  |
  | RGB-D 融合
  v
map.pcd
```

VINS-Fusion 本身不是稠密建图算法，它主要提供高质量相机位姿、回环和位姿图优化。稠密地图由 PCL 风格深度拼接或 Open3D TSDF 基于深度图和优化轨迹融合生成。

### 4.1 采集建图数据

启动 RealSense：

```bash
roslaunch realsense2_camera rs_camera.launch \
  enable_color:=true \
  enable_depth:=true \
  align_depth:=true \
  enable_sync:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=linear_interpolation \
  color_width:=640 color_height:=480 color_fps:=30 \
  depth_width:=640 depth_height:=480 depth_fps:=30
```

录制 rosbag：

```bash
rosbag record \
  /camera/color/image_raw \
  /camera/aligned_depth_to_color/image_raw \
  /camera/color/camera_info \
  /camera/imu \
  /tf \
  -O mapping.bag
```

建议采集方式：

- 低速移动，避免快速旋转。
- 同一区域尽量形成闭环。
- 避免强反光、纯白墙、玻璃等 RealSense 深度退化区域。
- 保持距离 0.5 m 到 5 m。

### 4.2 离线运行 VINS-Fusion

你已经在机载计算机上编译好 VINS-Fusion，因此只需要准备对应配置文件。

建议检查配置项：

```yaml
image_topic: "/camera/color/image_raw"
imu_topic: "/camera/imu"
output_path: "/home/nvidia/vins_output/"
loop_closure: 1
```

离线回放 rosbag：

```bash
roscore
```

新终端：

```bash
roslaunch vins vins_rviz.launch
```

新终端：

```bash
roslaunch vins_estimator realsense.launch
```

新终端：

```bash
rosbag play mapping.bag --clock
```

保存 VINS-Fusion 回环优化后的轨迹，建议格式：

```text
timestamp tx ty tz qx qy qz qw
```

建议命名：

```text
vins_loop_optimized.txt
```

### 4.3 使用 PCL 风格深度拼接生成 PCD

该方案不依赖 Open3D，直接把每帧 depth 投影成相机坐标点云，再使用 VINS 位姿转换到地图坐标，最后用体素网格融合输出 PCL 兼容的 `PCD` 文件。

本工作区已经提供脚本：

```text
scripts/pcl_depth_stitch_from_bag.py
```

输入轨迹格式：

```text
timestamp,px,py,pz,qw,qx,qy,qz,vx,vy,vz
```

运行示例：

```bash
python3 scripts/pcl_depth_stitch_from_bag.py \
  --bag mapping.bag \
  --traj vins_loop_optimized.txt \
  --out map_pcl_stitched.pcd \
  --depth-topic /camera/aligned_depth_to_color/image_raw \
  --camera-info-topic /camera/color/camera_info \
  --depth-min 0.20 \
  --depth-max 4.0 \
  --voxel-leaf 0.03 \
  --pixel-stride 2 \
  --frame-stride 1 \
  --min-voxel-points 2 \
  --paint-color 180,180,180
```

如果 VINS 输出的是 IMU/body 位姿，而不是相机位姿，需要提供相机外参 `T_body_cam`：

```bash
python3 scripts/pcl_depth_stitch_from_bag.py \
  --bag mapping.bag \
  --traj vins_loop_optimized.txt \
  --out map_pcl_stitched.pcd \
  --depth-topic /camera/aligned_depth_to_color/image_raw \
  --camera-info-topic /camera/color/camera_info \
  --body-t-cam body_T_cam0.yaml
```

推荐调参：

```text
voxel-leaf: 0.03 - 0.05 m
pixel-stride: 1 - 3
depth-max: 3.5 - 5.0 m
min-voxel-points: 2 - 5
```

如果点云太稀：

```bash
--pixel-stride 1 --min-voxel-points 1
```

如果噪声明显：

```bash
--voxel-leaf 0.05 --min-voxel-points 4 --depth-max 3.5
```

查看输出：

```bash
pcl_viewer map_pcl_stitched.pcd
```

或：

```bash
python3 -c "import open3d as o3d; p=o3d.io.read_point_cloud('map_pcl_stitched.pcd'); o3d.visualization.draw_geometries([p])"
```

### 4.4 使用 Open3D 融合生成 PCD

建议在地面站或开发主机上做离线融合，不建议在飞行时做。

安装 Open3D：

```bash
pip3 install open3d numpy opencv-python pyyaml
```

本工作区已经提供转换脚本：

```text
scripts/open3d_tsdf_from_bag.py
```

输入轨迹格式：

```text
timestamp,px,py,pz,qw,qx,qy,qz,vx,vy,vz
```

运行示例：

```bash
python3 open3d_tsdf.py \
  --bag mapping.bag \
  --traj vins_loop_optimized.txt \
  --out map.pcd \
  --color-topic /camera/color/image_raw \
  --depth-topic /camera/aligned_depth_to_color/image_raw \
  --camera-info-topic /camera/color/camera_info \
  --voxel-length 0.05 \
  --sdf-trunc 0.20 \
  --depth-trunc 5.0 \
  --frame-stride 3
```

如果 VINS 输出的是 IMU/body 位姿，而不是相机位姿，需要提供相机外参 `T_body_cam`：

```yaml
T_body_cam:
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
```

然后运行：

```bash
python3 scripts/open3d_tsdf_from_bag.py \
  --bag mapping.bag \
  --traj vins_loop_optimized.txt \
  --out map.pcd \
  --body-t-cam body_T_cam0.yaml
```

融合参数建议：

```text
voxel_length: 0.03 - 0.05 m
sdf_trunc: voxel_length * 4
depth_trunc: 4.0 - 6.0 m
frame_stride: 3 - 5
```

如果更重视显示效果：

```text
voxel_length = 0.02 - 0.03
```

如果更重视加载速度：

```text
voxel_length = 0.05 - 0.10
```

最终输出：

```text
map.pcd
map_meta.yaml
vins_loop_optimized.txt
```

推荐 `map_meta.yaml`：

```yaml
map_id: indoor_area_001
frame: map
pcd: map.pcd
trajectory: vins_loop_optimized.txt
voxel_size: 0.05
origin_xyz: [0.0, 0.0, 0.0]
origin_yaw_deg: 0.0
source_bag: mapping.bag
coordinate: ENU
```

## 5. 在线定位部署

### 5.1 启动 RealSense

```bash
roslaunch realsense2_camera rs_rgbd.launch \
  align_depth:=true \
  enable_sync:=true \
  enable_gyro:=true \
  enable_accel:=true \
  color_width:=640 color_height:=480 color_fps:=30 \
  depth_width:=640 depth_height:=480 depth_fps:=30
```

检查话题：

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/aligned_depth_to_color/image_raw
rostopic hz /camera/imu
```

### 5.2 启动 VINS-Fusion

```bash
roslaunch vins_estimator realsense.launch
```

检查 VINS 输出：

```bash
rostopic echo /vins_estimator/odometry
rostopic hz /vins_estimator/odometry
```

要求：

- 位姿输出频率稳定。
- 无人机静止时位置漂移较小。
- 快速转动时不频繁丢跟踪。

### 5.3 启动坐标桥接节点

需要实现一个 `vins_map_bridge` 节点，职责如下：

订阅：

```text
/vins_estimator/odometry
/initialpose 或自定义 /set_map_pose
/vins_loop/relocalization_pose，可选
```

维护：

```text
T_map_odom
```

发布：

```text
/tf: map -> odom_vins
/tf: odom_vins -> base_link
/drone_pose_in_map
/mavros/vision_pose/pose 或 MAVLink ODOMETRY
```

核心逻辑：

```text
1. VINS 连续输出 T_odom_base。
2. bridge 根据当前 T_map_odom 计算 T_map_base。
3. 飞控使用 T_odom_base。
4. 地面站使用 T_map_base。
5. 手动初始化、回环或重定位时，只更新 T_map_odom。
```

## 6. QGC / 地面站位置更新

### 6.1 如果 QGC 使用局部坐标

发布 MAVLink `ODOMETRY` 或 `LOCAL_POSITION_NED`。

坐标转换：

```text
ROS ENU:
x forward/east
y left/north
z up

MAVLink NED:
x north
y east
z down
```

注意 ENU 到 NED 的转换，不要直接把 ROS 坐标发送给飞控或 QGC。

### 6.2 如果 QGC 需要显示在经纬度地图上

需要设置地图原点：

```text
lat0, lon0, alt0
yaw_map_to_enu
```

然后：

```text
map xyz -> ENU -> WGS84 lat/lon/alt
```

再发送给 QGC。

室内无 GPS 场景下，建议 QGC 负责任务调度，点云地图由独立可视化程序显示。两者共享同一个 `map` 坐标系即可。

## 7. 手动初始化与重定位

### 7.1 手动初始化流程

地面站选择无人机在离线地图中的初始位置：

```text
x_map, y_map, z_map, yaw_map
```

发送给机载端：

```text
/set_initial_map_pose
```

机载端读取当前 VINS 位姿：

```text
T_odom_base_current
```

构造期望地图位姿：

```text
T_map_base_desired
```

更新：

```text
T_map_odom = T_map_base_desired * inverse(T_odom_base_current)
```

这样 QGC/地面站上的无人机位置会立即更新到地图中的指定位置。

### 7.2 VINS 回环或重定位后的更新

当 VINS-Fusion 产生回环或重定位结果时：

```text
T_map_base_relocalized
```

读取当前连续 VINS 位姿：

```text
T_odom_base_current
```

更新：

```text
T_map_odom = T_map_base_relocalized * inverse(T_odom_base_current)
```

注意：

- 不要重置飞控使用的 `odom_vins -> base_link`。
- QGC 可以看到位置修正。
- 任务调度层应暂停、更新当前位置、重新检查航点，然后继续。

## 8. 飞控接口建议

推荐分两路：

```text
连续 VINS odometry -> MAVROS -> 飞控 EKF
map pose -> QGC / 地面站任务层
```

PX4 常用接口：

```text
/mavros/vision_pose/pose
/mavros/odometry/out
```

ArduPilot 常用接口：

```text
VISION_POSITION_ESTIMATE
ODOMETRY
```

飞控融合参数需要按具体飞控配置，这里只给原则：

- 外部视觉输入必须连续。
- 不把回环跳变位姿直接送入 EKF。
- 时间戳必须正确。
- 坐标系必须从 ROS ENU 转为飞控需要的 NED 或 FRD。

## 9. 实时性建议

机载端推荐：

```text
RealSense: 640x480, 30 Hz
VINS-Fusion: 实时运行
回环检测: 低频或按需触发
点云地图: 不在机载端实时构建
QGC/地图显示: 地面站处理
```

Orin NX 上建议监控：

```bash
sudo tegrastats
```

推荐资源状态：

```text
CPU 平均低于 70%
内存至少保留 1.5 GB
VINS odometry 频率稳定
RealSense 图像无明显掉帧
```

如果实时性不足，优先调整：

```text
1. 降低图像分辨率到 640x480
2. 关闭机载端 RViz
3. 降低回环检测频率
4. 减少地面站高频点云传输
5. 将地图可视化完全放到地面站
```

## 10. 推荐部署顺序

### 阶段 1：验证在线 VINS

```text
1. 启动 RealSense。
2. 启动 VINS-Fusion。
3. 检查 /vins_estimator/odometry。
4. 手持或低速移动相机，确认轨迹连续。
```

### 阶段 2：采集离线建图 rosbag

```text
1. 低速采集完整环境。
2. 保证有闭环路径。
3. 保存 mapping.bag。
```

### 阶段 3：生成 PCD 地图

```text
1. 离线回放 rosbag。
2. VINS-Fusion 输出回环优化轨迹。
3. Open3D 融合 RGB-D。
4. 导出 map.pcd。
5. 保存 map_meta.yaml。
```

### 阶段 4：实现 vins_map_bridge

```text
1. 订阅 VINS odometry。
2. 发布 map -> odom_vins -> base_link。
3. 支持手动设置初始位置。
4. 支持重定位后更新 T_map_odom。
```

### 阶段 5：接入 QGC / 地面站

```text
1. QGC 负责任务调度。
2. 独立可视化程序加载 map.pcd。
3. 地面站订阅 /drone_pose_in_map。
4. 地图中显示无人机实时位置。
5. 手动重定位后验证位置更新。
```

### 阶段 6：接入飞控

```text
1. 将连续 VINS odometry 接入 MAVROS。
2. 飞控 EKF 融合外部视觉。
3. 室内低速悬停测试。
4. 验证回环或手动重定位不会导致飞控位置突跳。
```

## 11. 最终交付物

建议项目最终包含：

```text
map.pcd
map_meta.yaml
vins_loop_optimized.txt
vins_map_bridge ROS 节点
QGC/地面站地图显示模块
RealSense 启动文件
VINS-Fusion 配置文件
MAVROS 外部视觉配置
```

## 12. 关键风险

必须重点验证：

- RealSense 与 IMU 时间同步。
- VINS-Fusion 外参准确性。
- ROS ENU 与 MAVLink NED 坐标转换。
- 回环或重定位后的位姿跳变不进入飞控控制链路。
- QGC 和 PCD 地图使用同一个 `map` 坐标定义。
- 地面站任务点在重定位后需要重新计算相对无人机的位置。

## 13. 最终结论

该方案可行，并且适合无人机实际部署：

- VINS-Fusion 负责机载实时定位和回环。
- Open3D 离线融合生成高质量 PCD 地图。
- QGC 负责任务调度。
- 地面站地图程序负责稠密点云可视化。
- `vins_map_bridge` 负责把 VINS、离线地图、QGC 和飞控连接起来。

工程实现时最重要的是保持两套位姿：

```text
连续 odom 位姿：给飞控
重定位 map 位姿：给 QGC 和地面站
```

这样可以同时满足实时飞行稳定性和地面站高质量地图显示。
