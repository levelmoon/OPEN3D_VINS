# loop_fusion 手动先验与手动触发说明

## 已修改文件

```text
vins-fusion-gpu/vins-fusion-gpu/loop_fusion/src/pose_graph.h
vins-fusion-gpu/vins-fusion-gpu/loop_fusion/src/pose_graph.cpp
vins-fusion-gpu/vins-fusion-gpu/loop_fusion/src/pose_graph_node.cpp
vins-fusion-gpu/vins-fusion-gpu/vins_estimator/src/rosNodeTest.cpp
vins-fusion-gpu/vins-fusion-gpu/vins_estimator/src/utility/visualization.h
vins-fusion-gpu/vins-fusion-gpu/vins_estimator/src/utility/visualization.cpp
```

## 新增功能

### 1. 回环模式选择

加载历史 pose graph 后，终端会要求选择模式：

```text
0: auto global loop
1: manual prior loop
2: hybrid manual prior + global fallback
```

选择 `1` 或 `2` 后输入：

```text
x y yaw_deg radius yaw_radius_deg
```

示例：

```text
1.2 3.5 90 2.0 45
```

含义：

```text
x, y: 当前无人机在 pose graph/map 坐标系中的大概位置
yaw_deg: 当前大概航向，单位 degree
radius: 位置搜索半径，单位 m
yaw_radius_deg: 航向搜索范围，单位 degree
```

### 2. 手动触发重定位

新增 estimator service：

```text
/vins_fusion/force_keyframe
```

作用：

```text
强制 vins_estimator 发布当前滑窗最新帧的：
/vins_fusion/keyframe_pose
/vins_fusion/keyframe_point
```

调用方式：

```bash
rosservice call /vins_fusion/force_keyframe "{}"
```

新增 loop_fusion 键盘命令：

```text
r
```

作用：

```text
使用最近一次同步好的 image + keyframe_point + keyframe_pose，
强制构造一个 KeyFrame，
并调用 posegraph.addKeyFrame(keyframe, 1) 触发回环检测。
```

按 `r` 时会自动执行：

```text
1. loop_fusion 调用 /vins_fusion/force_keyframe，让 estimator 强制发布当前帧。
2. loop_fusion 收到并缓存这组 image/point/pose。
3. loop_fusion 强制构造 KeyFrame 并触发回环检测。
```

这样即使无人机原地不动、VINS 没有自动产生关键帧，也可以主动尝试重定位。

原有命令保留：

```text
s: 保存 pose graph 并退出
n: 新建 sequence
```

## 代码修改摘要

### pose_graph.h / pose_graph.cpp

新增三种模式：

```cpp
LOOP_AUTO
LOOP_MANUAL_PRIOR
LOOP_HYBRID
```

原始 `detectLoop()` 逻辑保留为：

```cpp
detectLoopAuto()
```

新增：

```cpp
detectLoopManual()
setLoopMode()
setManualPrior()
buildManualCandidateDatabase()
```

手动先验模式会根据输入的 `x,y,yaw,radius,yaw_radius` 筛选历史关键帧，构建 `manual_db`，只在候选关键帧中做 BoW 查询。

### pose_graph_node.cpp

新增：

```cpp
select_loop_mode_after_load()
add_keyframe_from_msgs()
latest_reloc_image_msg
latest_reloc_point_msg
latest_reloc_pose_msg
```

`process()` 每次同步到 image/point/pose 后会缓存最近一帧。

按 `r` 时会使用缓存帧强制添加关键帧并触发回环检测。

缓存发生在 `SKIP_FIRST_CNT` 和 `SKIP_CNT` 判断之前，所以即使自动流程跳过了最开始的关键帧，`r` 也能使用最近缓存的同步帧。

手动模式不使用原始自动模式的 `frame_index > 50` 限制，因此加载历史 pose graph 后可以立即尝试重定位。

### vins_estimator

新增：

```cpp
pubForcedKeyframe()
force_keyframe_callback()
```

`pubForcedKeyframe()` 使用 `WINDOW_SIZE - 1` 发布当前滑窗最新帧，不再依赖 `marginalization_flag == 0`。

## 使用步骤

1. 确保配置文件加载历史 pose graph：

```yaml
load_previous_pose_graph: 1
```

2. 编译：

```bash
catkin_make
source devel/setup.bash
```

3. 启动：

```bash
rosrun loop_fusion loop_fusion_node your_config.yaml
```

4. 推荐选择混合模式：

```text
2
```

5. 输入大概位置：

```text
x y yaw_deg radius yaw_radius_deg
```

6. 地面静止时，如果没有自动回环，在 `loop_fusion` 终端按：

```text
r
```

## 预期输出

选择手动或混合模式：

```text
[loop_fusion] manual prior: ...
[loop_fusion] manual candidate keyframes: N
```

按 `r` 后：

```text
[loop_fusion] force_keyframe service: forced keyframe published
[vins_estimator] forced keyframe published at index ...
[loop_fusion] manual trigger keyframe index=...
[loop_fusion] manual relocalization trigger finished
```

如果检测到手动候选：

```text
[loop_fusion] manual prior loop candidate: current=... old=... score=...
```

## 注意事项

```text
1. x,y,yaw 必须是已加载 pose graph/map 坐标系，不是当前 VINS 局部坐标。
2. yaw 单位是 degree。
3. 手动先验只用于缩小候选范围，不会直接强制添加回环边。
4. r 命令需要 loop_fusion 已经收到过至少一组同步的 image/point/pose。
5. 如果 manual candidate keyframes 为 0，增大 radius 或 yaw_radius。
6. 建议优先使用模式 2，避免手动先验不准时完全无法回环。
7. 如果 `/vins_fusion/force_keyframe` 返回 estimator is not NON_LINEAR yet，说明 VINS 还没有初始化完成，不能重定位。
8. 如果 loop_fusion 提示 force_keyframe service unavailable，确认 vins_estimator 已经重新编译并启动。
```
