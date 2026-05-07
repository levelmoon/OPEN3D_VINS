#!/usr/bin/env python3
import argparse
import bisect
import csv
import math
import os

import numpy as np
import open3d as o3d
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
    t = np.array([px, py, pz], dtype=np.float64)
    r = quat_wxyz_to_rot(qw, qx, qy, qz)
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = r
    mat[:3, 3] = t
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
    print(f"trajectory timestamps scaled by {scale:g}")
    print(f"trajectory time range: {times[0]:.6f} -> {times[-1]:.6f}")
    order = np.argsort(times)
    return [times[i] for i in order], [poses[i] for i in order]


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
    mat = mat.reshape(4, 4)
    return mat


def camera_info_to_intrinsic(msg):
    return o3d.camera.PinholeCameraIntrinsic(
        width=msg.width,
        height=msg.height,
        fx=msg.K[0],
        fy=msg.K[4],
        cx=msg.K[2],
        cy=msg.K[5],
    )


def get_stamp(msg):
    return msg.header.stamp.to_sec()


def normalize_depth_image(depth_cv, encoding):
    # RealSense depth is usually 16UC1 in millimeters. Some pipelines publish
    # 32FC1 in meters. Open3D accepts uint16 or float32 depth images.
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


def apply_depth_limits(depth_cv, depth_scale, depth_min, depth_trunc):
    if depth_min <= 0:
        return depth_cv
    limited = depth_cv.copy()
    if np.issubdtype(limited.dtype, np.integer):
        min_raw = int(depth_min * depth_scale)
        max_raw = int(depth_trunc * depth_scale) if depth_trunc > 0 else np.iinfo(limited.dtype).max
        limited[(limited > 0) & (limited < min_raw)] = 0
        limited[limited > max_raw] = 0
    else:
        limited[(limited > 0.0) & (limited < depth_min)] = 0.0
        if depth_trunc > 0:
            limited[limited > depth_trunc] = 0.0
    return np.ascontiguousarray(limited)


def cleaned_point_cloud(pcd, args):
    pcd.remove_non_finite_points()
    print(f"raw points={len(pcd.points)}")

    if args.save_raw:
        raw_path = args.raw_out
        if raw_path is None:
            root, ext = os.path.splitext(args.out)
            raw_path = f"{root}_raw{ext or '.pcd'}"
        o3d.io.write_point_cloud(raw_path, pcd, write_ascii=False, compressed=False)
        print(f"saved raw {raw_path}")

    if args.no_filter:
        return pcd

    if args.stat_nb_neighbors > 0 and args.stat_std_ratio > 0:
        before = len(pcd.points)
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=args.stat_nb_neighbors,
            std_ratio=args.stat_std_ratio,
        )
        print(f"statistical filter: {before} -> {len(pcd.points)}")

    if args.radius_nb_points > 0 and args.radius > 0:
        before = len(pcd.points)
        pcd, _ = pcd.remove_radius_outlier(
            nb_points=args.radius_nb_points,
            radius=args.radius,
        )
        print(f"radius filter: {before} -> {len(pcd.points)}")

    if args.pcd_voxel_downsample > 0:
        before = len(pcd.points)
        pcd = pcd.voxel_down_sample(args.pcd_voxel_downsample)
        print(f"voxel downsample: {before} -> {len(pcd.points)}")

    pcd.remove_non_finite_points()
    return pcd


def turbo_colormap(values):
    # Polynomial approximation of Google's Turbo colormap for values in [0, 1].
    x = np.clip(values, 0.0, 1.0).reshape(-1, 1)
    coeffs = np.array(
        [
            [0.13572138, 4.61539260, -42.66032258, 132.13108234, -152.94239396, 59.28637943],
            [0.09140261, 2.19418839, 4.84296658, -14.18503333, 4.27729857, 2.82956604],
            [0.10667330, 12.64194608, -60.58204836, 110.36276771, -89.90310912, 27.34824973],
        ],
        dtype=np.float64,
    )
    powers = np.hstack([x**i for i in range(6)])
    colors = powers @ coeffs.T
    return np.clip(colors, 0.0, 1.0)


def apply_output_coloring(pcd, args):
    if args.color_mode == "rgb":
        return pcd

    points = np.asarray(pcd.points)
    if len(points) == 0:
        return pcd

    if args.color_mode == "uniform":
        color = parse_rgb_color(args.paint_color)
        pcd.paint_uniform_color(color)
        return pcd

    axis_index = {"x": 0, "y": 1, "z": 2}[args.height_axis]
    h = points[:, axis_index]
    h_min = float(np.min(h)) if args.height_min is None else args.height_min
    h_max = float(np.max(h)) if args.height_max is None else args.height_max
    if h_max <= h_min:
        normalized = np.zeros_like(h, dtype=np.float64)
    else:
        normalized = (h - h_min) / (h_max - h_min)

    if args.invert_height_color:
        normalized = 1.0 - normalized

    colors = turbo_colormap(normalized)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    print(f"height color axis={args.height_axis}, min={h_min:.3f}, max={h_max:.3f}")
    return pcd


def parse_rgb_color(text):
    values = [float(x.strip()) for x in text.split(",")]
    if len(values) != 3:
        raise ValueError("--paint-color must be formatted as r,g,b")
    if max(values) > 1.0:
        values = [v / 255.0 for v in values]
    return [min(1.0, max(0.0, v)) for v in values]


def main():
    parser = argparse.ArgumentParser(description="Fuse RealSense RGB-D rosbag and VINS trajectory into a PCD map.")
    parser.add_argument("--bag", required=True, help="Input rosbag path.")
    parser.add_argument("--traj", required=True, help="VINS trajectory: timestamp,px,py,pz,qw,qx,qy,qz,vx,vy,vz")
    parser.add_argument(
        "--traj-time-scale",
        type=float,
        default=None,
        help="Scale applied to trajectory timestamps. Use 1e-9 for nanoseconds, 1e-6 for microseconds, 1e-3 for milliseconds.",
    )
    parser.add_argument("--out", default="map.pcd", help="Output PCD path.")
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--body-t-cam", default=None, help="YAML containing 4x4 T_body_cam. Defaults to identity.")
    parser.add_argument("--voxel-length", type=float, default=0.03)
    parser.add_argument("--sdf-trunc", type=float, default=0.12)
    parser.add_argument("--depth-min", type=float, default=0.20)
    parser.add_argument("--depth-trunc", type=float, default=4.0)
    parser.add_argument("--depth-scale", type=float, default=1000.0)
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--max-sync-dt", type=float, default=0.20)
    parser.add_argument("--max-pose-dt", type=float, default=0.20)
    parser.add_argument("--pcd-voxel-downsample", type=float, default=0.01)
    parser.add_argument("--no-filter", action="store_true", help="Disable statistical/radius filtering.")
    parser.add_argument("--save-raw", action="store_true", help="Also save the unfiltered extracted point cloud.")
    parser.add_argument("--raw-out", default=None, help="Raw point cloud output path. Used only with --save-raw.")
    parser.add_argument("--stat-nb-neighbors", type=int, default=20)
    parser.add_argument("--stat-std-ratio", type=float, default=1.5)
    parser.add_argument("--radius-nb-points", type=int, default=6)
    parser.add_argument("--radius", type=float, default=0.08)
    parser.add_argument(
        "--color-mode",
        choices=["rgb", "height", "uniform"],
        default="height",
        help="Output coloring. rgb keeps camera color, height overwrites colors by height, uniform paints one color.",
    )
    parser.add_argument("--height-axis", choices=["x", "y", "z"], default="z")
    parser.add_argument("--height-min", type=float, default=None)
    parser.add_argument("--height-max", type=float, default=None)
    parser.add_argument("--invert-height-color", action="store_true")
    parser.add_argument("--paint-color", default="180,180,180", help="Uniform color used when --color-mode uniform.")
    args = parser.parse_args()

    if args.frame_stride < 1:
        raise ValueError("--frame-stride must be >= 1")

    traj_times, traj_poses = read_trajectory(args.traj, args.traj_time_scale)
    t_body_cam = read_body_to_camera(args.body_t_cam)

    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=args.voxel_length,
        sdf_trunc=args.sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    bridge = CvBridge()
    intrinsic = None
    latest_color = None
    latest_depth = None
    integrated = 0
    skipped = 0
    seen_depth = 0
    printed_depth_debug = False
    topic_counts = {args.color_topic: 0, args.depth_topic: 0, args.camera_info_topic: 0}
    skip_reasons = {
        "missing_intrinsic_or_color": 0,
        "color_depth_dt": 0,
        "pose_dt": 0,
    }

    topics = [args.color_topic, args.depth_topic, args.camera_info_topic]
    with rosbag.Bag(args.bag, "r") as bag:
        print(f"bag time range: {bag.get_start_time():.6f} -> {bag.get_end_time():.6f}")
        available_topics = sorted(bag.get_type_and_topic_info().topics.keys())
        missing_topics = [topic for topic in topics if topic not in available_topics]
        if missing_topics:
            print("missing requested topics:")
            for topic in missing_topics:
                print(f"  {topic}")
            print("available image/camera topics:")
            for topic in available_topics:
                if "camera" in topic or "image" in topic or "depth" in topic or "infra" in topic:
                    print(f"  {topic}")
        for topic, msg, _ in bag.read_messages(topics=topics):
            if topic in topic_counts:
                topic_counts[topic] += 1
            if topic == args.camera_info_topic:
                if intrinsic is None:
                    intrinsic = camera_info_to_intrinsic(msg)
                    print("camera intrinsic:", intrinsic.intrinsic_matrix)
                continue

            if topic == args.color_topic:
                latest_color = msg
                continue

            if topic != args.depth_topic:
                continue

            seen_depth += 1
            if seen_depth % args.frame_stride != 0:
                continue
            if intrinsic is None or latest_color is None:
                skipped += 1
                skip_reasons["missing_intrinsic_or_color"] += 1
                continue

            depth_stamp = get_stamp(msg)
            color_stamp = get_stamp(latest_color)
            if abs(depth_stamp - color_stamp) > args.max_sync_dt:
                skipped += 1
                skip_reasons["color_depth_dt"] += 1
                continue

            t_world_body, pose_dt = nearest_pose(traj_times, traj_poses, depth_stamp, args.max_pose_dt)
            if t_world_body is None:
                skipped += 1
                skip_reasons["pose_dt"] += 1
                continue

            color_cv = bridge.imgmsg_to_cv2(latest_color, desired_encoding="rgb8")
            depth_cv = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            depth_cv = normalize_depth_image(depth_cv, msg.encoding)
            depth_cv = apply_depth_limits(depth_cv, args.depth_scale, args.depth_min, args.depth_trunc)
            if not printed_depth_debug:
                print(
                    "depth debug:",
                    f"encoding={msg.encoding}",
                    f"dtype={depth_cv.dtype}",
                    f"shape={depth_cv.shape}",
                    f"contiguous={depth_cv.flags['C_CONTIGUOUS']}",
                    f"min={np.nanmin(depth_cv)}",
                    f"max={np.nanmax(depth_cv)}",
                )
                printed_depth_debug = True

            color_o3d = o3d.geometry.Image(np.ascontiguousarray(color_cv))
            depth_o3d = o3d.geometry.Image(depth_cv)
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_o3d,
                depth_o3d,
                depth_scale=args.depth_scale,
                depth_trunc=args.depth_trunc,
                convert_rgb_to_intensity=False,
            )

            t_world_cam = t_world_body @ t_body_cam
            t_cam_world = np.linalg.inv(t_world_cam)
            volume.integrate(rgbd, intrinsic, t_cam_world)
            integrated += 1

            if integrated % 50 == 0:
                print(f"integrated={integrated}, skipped={skipped}, last_pose_dt={pose_dt:.4f}s")

    if integrated == 0:
        print("topic message counts:")
        for topic, count in topic_counts.items():
            print(f"  {topic}: {count}")
        print("skip reasons:")
        for reason, count in skip_reasons.items():
            print(f"  {reason}: {count}")
        raise RuntimeError("no frames integrated; check topics, timestamps, trajectory, and sync thresholds")

    pcd = cleaned_point_cloud(volume.extract_point_cloud(), args)
    pcd = apply_output_coloring(pcd, args)

    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    o3d.io.write_point_cloud(args.out, pcd, write_ascii=False, compressed=False)
    print(f"saved {args.out}")
    print(f"points={len(pcd.points)}, integrated={integrated}, skipped={skipped}")


if __name__ == "__main__":
    main()
