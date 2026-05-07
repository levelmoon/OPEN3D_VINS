#!/usr/bin/env python3
import argparse
import bisect
import csv
import math
import os
import struct

import numpy as np
import rosbag
import yaml
from cv_bridge import CvBridge


def quat_wxyz_to_rot(qw, qx, qy, qz):
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n == 0.0:
        raise ValueError("zero quaternion")
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def pose_to_matrix(px, py, pz, qw, qx, qy, qz):
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = quat_wxyz_to_rot(qw, qx, qy, qz)
    mat[:3, 3] = np.array([px, py, pz], dtype=np.float64)
    return mat


def infer_time_scale(stamps):
    if not stamps:
        return 1.0
    median_stamp = float(np.median(np.array(stamps, dtype=np.float64)))
    if median_stamp > 1.0e17:
        return 1.0e-9
    if median_stamp > 1.0e14:
        return 1.0e-6
    if median_stamp > 1.0e11:
        return 1.0e-3
    return 1.0


def read_trajectory(path, time_scale=None):
    times = []
    poses = []
    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            row = [x.strip() for x in row if x.strip() != ""]
            if not row or row[0].startswith("#"):
                continue
            if row[0].lower() in ("timestamp", "time", "stamp"):
                continue
            if len(row) < 8:
                raise ValueError("trajectory row must have at least 8 columns: timestamp,px,py,pz,qw,qx,qy,qz")
            stamp = float(row[0])
            px, py, pz = float(row[1]), float(row[2]), float(row[3])
            qw, qx, qy, qz = float(row[4]), float(row[5]), float(row[6]), float(row[7])
            times.append(stamp)
            poses.append(pose_to_matrix(px, py, pz, qw, qx, qy, qz))
    if not times:
        raise ValueError("empty trajectory")
    scale = infer_time_scale(times) if time_scale is None else time_scale
    times = [t * scale for t in times]
    order = np.argsort(times)
    times = [times[i] for i in order]
    poses = [poses[i] for i in order]
    print(f"trajectory timestamps scaled by {scale:g}")
    print(f"trajectory time range: {times[0]:.6f} -> {times[-1]:.6f}")
    return times, poses


def nearest_pose(times, poses, stamp, max_dt):
    idx = bisect.bisect_left(times, stamp)
    candidates = []
    if idx < len(times):
        candidates.append(idx)
    if idx > 0:
        candidates.append(idx - 1)
    if not candidates:
        return None, None
    best = min(candidates, key=lambda i: abs(times[i] - stamp))
    dt = abs(times[best] - stamp)
    if dt > max_dt:
        return None, dt
    return poses[best], dt


def read_body_to_camera(path):
    if path is None:
        return np.eye(4, dtype=np.float64)
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    if "T_body_cam" in data:
        mat = np.array(data["T_body_cam"], dtype=np.float64)
    elif "body_T_cam" in data:
        mat = np.array(data["body_T_cam"], dtype=np.float64)
    else:
        mat = np.array(data, dtype=np.float64)
    return mat.reshape(4, 4)


def normalize_depth(depth_cv, encoding):
    depth_cv = np.asarray(depth_cv)
    if depth_cv.ndim == 3 and depth_cv.shape[2] == 1:
        depth_cv = depth_cv[:, :, 0]
    if depth_cv.dtype.byteorder not in ("=", "|"):
        depth_cv = depth_cv.byteswap().newbyteorder()
    if encoding in ("16UC1", "mono16"):
        return np.ascontiguousarray(depth_cv, dtype=np.uint16)
    if encoding == "32FC1":
        return np.ascontiguousarray(depth_cv, dtype=np.float32)
    if depth_cv.dtype == np.uint16:
        return np.ascontiguousarray(depth_cv)
    if depth_cv.dtype in (np.float32, np.float64):
        return np.ascontiguousarray(depth_cv, dtype=np.float32)
    if np.issubdtype(depth_cv.dtype, np.integer):
        clipped = np.clip(depth_cv, 0, np.iinfo(np.uint16).max)
        return np.ascontiguousarray(clipped, dtype=np.uint16)
    raise ValueError(f"unsupported depth encoding={encoding}, dtype={depth_cv.dtype}")


def depth_to_points(depth, k, depth_scale, depth_min, depth_max, pixel_stride):
    fx, fy = k[0], k[4]
    cx, cy = k[2], k[5]
    h, w = depth.shape

    vv, uu = np.mgrid[0:h:pixel_stride, 0:w:pixel_stride]
    sampled = depth[0:h:pixel_stride, 0:w:pixel_stride]
    z = sampled.astype(np.float32)
    if np.issubdtype(depth.dtype, np.integer):
        z /= float(depth_scale)

    valid = np.isfinite(z) & (z >= depth_min) & (z <= depth_max)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64)

    u = uu[valid].astype(np.float32)
    v = vv[valid].astype(np.float32)
    z = z[valid]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack((x, y, z), axis=1).astype(np.float64)


def update_voxels(voxel_sum, voxel_count, points, leaf_size):
    if len(points) == 0:
        return
    keys = np.floor(points / leaf_size).astype(np.int64)
    unique_keys, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    sums = np.zeros((len(unique_keys), 3), dtype=np.float64)
    np.add.at(sums, inverse, points)

    for key, point_sum, count in zip(unique_keys, sums, counts):
        packed_key = (int(key[0]), int(key[1]), int(key[2]))
        if packed_key in voxel_sum:
            voxel_sum[packed_key] += point_sum
            voxel_count[packed_key] += int(count)
        else:
            voxel_sum[packed_key] = point_sum
            voxel_count[packed_key] = int(count)


def voxels_to_points(voxel_sum, voxel_count, min_points):
    points = []
    for key, point_sum in voxel_sum.items():
        count = voxel_count[key]
        if count >= min_points:
            points.append(point_sum / count)
    if not points:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(points, dtype=np.float32)


def parse_rgb(text):
    values = [float(x.strip()) for x in text.split(",")]
    if len(values) != 3:
        raise ValueError("--paint-color must be r,g,b")
    if all(v < 0 for v in values):
        return None
    if max(values) > 1.0:
        values = [v / 255.0 for v in values]
    values = [min(1.0, max(0.0, v)) for v in values]
    r, g, b = [int(round(v * 255.0)) for v in values]
    return (r << 16) | (g << 8) | b


def write_pcd(path, points, rgb=None, binary=True):
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    points = np.asarray(points, dtype=np.float32)
    n = len(points)
    if rgb is None:
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {n}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {n}\n"
            f"DATA {'binary' if binary else 'ascii'}\n"
        )
        if binary:
            with open(path, "wb") as f:
                f.write(header.encode("ascii"))
                f.write(points.astype(np.float32).tobytes())
        else:
            with open(path, "w") as f:
                f.write(header)
                np.savetxt(f, points, fmt="%.6f")
        return

    rgb_float = np.array([rgb], dtype=np.uint32).view(np.float32)[0]
    rgb_col = np.full((n, 1), rgb_float, dtype=np.float32)
    data = np.hstack((points, rgb_col)).astype(np.float32)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z rgb\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA {'binary' if binary else 'ascii'}\n"
    )
    if binary:
        with open(path, "wb") as f:
            f.write(header.encode("ascii"))
            f.write(data.tobytes())
    else:
        with open(path, "w") as f:
            f.write(header)
            np.savetxt(f, data, fmt="%.6f %.6f %.6f %.8e")


def main():
    parser = argparse.ArgumentParser(description="Stitch depth frames with VINS trajectory and write a PCL-compatible PCD.")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--traj", required=True)
    parser.add_argument("--out", default="map_pcl_stitched.pcd")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--body-t-cam", default=None)
    parser.add_argument("--traj-time-scale", type=float, default=None)
    parser.add_argument("--max-pose-dt", type=float, default=0.20)
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--pixel-stride", type=int, default=2)
    parser.add_argument("--depth-scale", type=float, default=1000.0)
    parser.add_argument("--depth-min", type=float, default=0.20)
    parser.add_argument("--depth-max", type=float, default=4.0)
    parser.add_argument("--voxel-leaf", type=float, default=0.03)
    parser.add_argument("--min-voxel-points", type=int, default=2)
    parser.add_argument("--paint-color", default="180,180,180", help="Uniform r,g,b. Use -1,-1,-1 to omit RGB field.")
    parser.add_argument("--ascii", action="store_true", help="Write ASCII PCD instead of binary.")
    args = parser.parse_args()

    if args.frame_stride < 1:
        raise ValueError("--frame-stride must be >= 1")
    if args.pixel_stride < 1:
        raise ValueError("--pixel-stride must be >= 1")
    if args.voxel_leaf <= 0:
        raise ValueError("--voxel-leaf must be > 0")

    traj_times, traj_poses = read_trajectory(args.traj, args.traj_time_scale)
    t_body_cam = read_body_to_camera(args.body_t_cam)
    rgb = parse_rgb(args.paint_color)

    bridge = CvBridge()
    k = None
    voxel_sum = {}
    voxel_count = {}
    integrated = 0
    skipped = 0
    seen_depth = 0
    skip_reasons = {"missing_intrinsic": 0, "pose_dt": 0, "empty_depth": 0}

    with rosbag.Bag(args.bag, "r") as bag:
        print(f"bag time range: {bag.get_start_time():.6f} -> {bag.get_end_time():.6f}")
        available_topics = sorted(bag.get_type_and_topic_info().topics.keys())
        for required in (args.depth_topic, args.camera_info_topic):
            if required not in available_topics:
                print(f"missing topic: {required}")

        for topic, msg, _ in bag.read_messages(topics=[args.depth_topic, args.camera_info_topic]):
            if topic == args.camera_info_topic:
                if k is None:
                    k = np.asarray(msg.K, dtype=np.float64)
                    print("camera K:", k.reshape(3, 3))
                continue

            if topic != args.depth_topic:
                continue

            seen_depth += 1
            if seen_depth % args.frame_stride != 0:
                continue
            if k is None:
                skipped += 1
                skip_reasons["missing_intrinsic"] += 1
                continue

            stamp = msg.header.stamp.to_sec()
            t_world_body, pose_dt = nearest_pose(traj_times, traj_poses, stamp, args.max_pose_dt)
            if t_world_body is None:
                skipped += 1
                skip_reasons["pose_dt"] += 1
                continue

            depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            depth = normalize_depth(depth, msg.encoding)
            cam_points = depth_to_points(
                depth,
                k,
                args.depth_scale,
                args.depth_min,
                args.depth_max,
                args.pixel_stride,
            )
            if len(cam_points) == 0:
                skipped += 1
                skip_reasons["empty_depth"] += 1
                continue

            t_world_cam = t_world_body @ t_body_cam
            world_points = (t_world_cam[:3, :3] @ cam_points.T).T + t_world_cam[:3, 3]
            update_voxels(voxel_sum, voxel_count, world_points, args.voxel_leaf)
            integrated += 1

            if integrated % 50 == 0:
                print(
                    f"integrated={integrated}, skipped={skipped}, "
                    f"voxels={len(voxel_count)}, last_pose_dt={pose_dt:.4f}s"
                )

    points = voxels_to_points(voxel_sum, voxel_count, args.min_voxel_points)
    if len(points) == 0:
        print("skip reasons:")
        for reason, count in skip_reasons.items():
            print(f"  {reason}: {count}")
        raise RuntimeError("no output points; check topics, trajectory time, depth range, and voxel filters")

    write_pcd(args.out, points, rgb=rgb, binary=not args.ascii)
    print(f"saved {args.out}")
    print(f"points={len(points)}, integrated={integrated}, skipped={skipped}, voxels={len(voxel_count)}")


if __name__ == "__main__":
    main()
