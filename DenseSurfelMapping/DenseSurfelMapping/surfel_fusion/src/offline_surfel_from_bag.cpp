#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <signal.h>
#include <unistd.h>

#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <surfel_map.h>

struct Args
{
    std::string bag = "mapping_mavros.bag";
    std::string traj = "vio_loop.txt";
    std::string out = "map_surfel_final_test.pcd";
    std::string depth_topic = "/camera/depth/image_rect_raw";
    std::string camera_info_topic = "/camera/depth/camera_info";
    std::string height_axis = "z";
    double traj_time_scale = 0.0;
    double traj_time_offset = 0.0;
    double max_pose_dt = 0.05;
    double depth_scale = 1000.0;
    double fuse_near = 0.10;
    double fuse_far = 3.0;
    int frame_stride = 1;
    int max_frames = 0;
    int min_valid_depth_pixels = 500;
    int min_update_times = 5;
    double start_time = -std::numeric_limits<double>::infinity();
    double end_time = std::numeric_limits<double>::infinity();
    double skip_first_sec = 10.0;
    double skip_last_sec = 5.0;
    double min_camera_z = -std::numeric_limits<double>::infinity();
    double max_camera_z = std::numeric_limits<double>::infinity();
    double max_angular_rate = 0.3;
    std::string vins_config_yaml = "../../vins-fusion-gpu/config/FS-J200/FS-J200_stereo_imu_config.yaml";
    double stereo_depth_tx = -0.025;
    double stereo_depth_ty = 0.0;
    double stereo_depth_tz = 0.0;
    double trajectory_keep_radius = 2.5;
    double density_voxel_size = 0.20;
    int density_min_neighbors = 4;
    bool remove_low_height_outliers = true;
    bool shutdown_roslaunch_on_finish = false;
    double low_height_percentile = 0.02;
    double low_height_margin = 0.05;
    Eigen::Vector3f crop_min = Eigen::Vector3f(
        -std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity());
    Eigen::Vector3f crop_max = Eigen::Vector3f(
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity());
    std::vector<Eigen::Vector2f> crop_polygon;
};

struct Trajectory
{
    std::vector<double> times;
    std::vector<Eigen::Matrix4f> poses;
};

static std::string get_arg(int argc, char **argv, const std::string &name, const std::string &default_value = "")
{
    for(int i = 1; i + 1 < argc; i++)
    {
        if(argv[i] == name)
            return argv[i + 1];
    }
    return default_value;
}

static bool has_arg(int argc, char **argv, const std::string &name)
{
    for(int i = 1; i < argc; i++)
    {
        if(argv[i] == name)
            return true;
    }
    return false;
}

static void print_usage()
{
    std::cout
        << "Usage: roslaunch surfel_fusion offline_surfel_from_bag.launch\n"
        << "       rosrun surfel_fusion offline_surfel_from_bag [--bag mapping_mavros.bag] [--traj vio_loop.txt]\n"
        << "Parameters are normally loaded from config/offline_surfel_from_bag.yaml.\n";
}

static bool parse_optional_double(int argc, char **argv, const std::string &name, double &target)
{
    std::string value = get_arg(argc, argv, name);
    if(value.empty())
        return false;
    target = std::stod(value);
    return true;
}

static std::vector<Eigen::Vector2f> parse_polygon(const std::string &text)
{
    std::vector<Eigen::Vector2f> polygon;
    if(text.empty())
        return polygon;

    std::stringstream ss(text);
    std::string token;
    while(std::getline(ss, token, ';'))
    {
        if(token.empty())
            continue;
        std::replace(token.begin(), token.end(), ',', ' ');
        std::stringstream point_stream(token);
        float x, y;
        if(!(point_stream >> x >> y))
            throw std::runtime_error("--crop-polygon must be formatted as x1,y1;x2,y2;x3,y3;...");
        polygon.emplace_back(x, y);
    }
    if(!polygon.empty() && polygon.size() < 3)
        throw std::runtime_error("--crop-polygon requires at least 3 vertices");
    return polygon;
}

static std::string resolve_package_relative_path(const std::string &path)
{
    if(path.empty())
        return path;
    if(path[0] == '/')
        return path;

    const std::string package_path = ros::package::getPath("surfel_fusion");
    if(package_path.empty())
        return path;
    return package_path + "/" + path;
}

static void load_ros_params(Args &args)
{
    ros::NodeHandle nh("~");
    nh.param<std::string>("bag", args.bag, args.bag);
    nh.param<std::string>("traj", args.traj, args.traj);
    nh.param<std::string>("out", args.out, args.out);
    nh.param<std::string>("depth_topic", args.depth_topic, args.depth_topic);
    nh.param<std::string>("camera_info_topic", args.camera_info_topic, args.camera_info_topic);
    nh.param<std::string>("height_axis", args.height_axis, args.height_axis);
    nh.param("traj_time_scale", args.traj_time_scale, args.traj_time_scale);
    nh.param("traj_time_offset", args.traj_time_offset, args.traj_time_offset);
    nh.param("max_pose_dt", args.max_pose_dt, args.max_pose_dt);
    nh.param("depth_scale", args.depth_scale, args.depth_scale);
    nh.param("fuse_near", args.fuse_near, args.fuse_near);
    nh.param("fuse_far", args.fuse_far, args.fuse_far);
    nh.param("frame_stride", args.frame_stride, args.frame_stride);
    nh.param("max_frames", args.max_frames, args.max_frames);
    nh.param("min_valid_depth_pixels", args.min_valid_depth_pixels, args.min_valid_depth_pixels);
    nh.param("min_update_times", args.min_update_times, args.min_update_times);
    nh.param("start_time", args.start_time, args.start_time);
    nh.param("end_time", args.end_time, args.end_time);
    nh.param("skip_first_sec", args.skip_first_sec, args.skip_first_sec);
    nh.param("skip_last_sec", args.skip_last_sec, args.skip_last_sec);
    nh.param("min_camera_z", args.min_camera_z, args.min_camera_z);
    nh.param("max_camera_z", args.max_camera_z, args.max_camera_z);
    nh.param("max_angular_rate", args.max_angular_rate, args.max_angular_rate);
    nh.param<std::string>("vins_config_yaml", args.vins_config_yaml, args.vins_config_yaml);
    nh.param("stereo_depth_tx", args.stereo_depth_tx, args.stereo_depth_tx);
    nh.param("stereo_depth_ty", args.stereo_depth_ty, args.stereo_depth_ty);
    nh.param("stereo_depth_tz", args.stereo_depth_tz, args.stereo_depth_tz);
    nh.param("trajectory_keep_radius", args.trajectory_keep_radius, args.trajectory_keep_radius);
    nh.param("density_voxel_size", args.density_voxel_size, args.density_voxel_size);
    nh.param("density_min_neighbors", args.density_min_neighbors, args.density_min_neighbors);
    nh.param("remove_low_height_outliers", args.remove_low_height_outliers, args.remove_low_height_outliers);
    nh.param("shutdown_roslaunch_on_finish", args.shutdown_roslaunch_on_finish, args.shutdown_roslaunch_on_finish);
    nh.param("low_height_percentile", args.low_height_percentile, args.low_height_percentile);
    nh.param("low_height_margin", args.low_height_margin, args.low_height_margin);
    std::string crop_polygon;
    nh.param<std::string>("crop_polygon", crop_polygon, "");
    if(!crop_polygon.empty())
        args.crop_polygon = parse_polygon(crop_polygon);
    double crop_value;
    if(nh.getParam("crop_x_min", crop_value))
        args.crop_min(0) = static_cast<float>(crop_value);
    if(nh.getParam("crop_y_min", crop_value))
        args.crop_min(1) = static_cast<float>(crop_value);
    if(nh.getParam("crop_z_min", crop_value))
        args.crop_min(2) = static_cast<float>(crop_value);
    if(nh.getParam("crop_x_max", crop_value))
        args.crop_max(0) = static_cast<float>(crop_value);
    if(nh.getParam("crop_y_max", crop_value))
        args.crop_max(1) = static_cast<float>(crop_value);
    if(nh.getParam("crop_z_max", crop_value))
        args.crop_max(2) = static_cast<float>(crop_value);
}

static Args parse_args(int argc, char **argv)
{
    Args args;
    if(has_arg(argc, argv, "--help") || has_arg(argc, argv, "-h"))
    {
        print_usage();
        std::exit(0);
    }

    load_ros_params(args);

    args.bag = get_arg(argc, argv, "--bag", args.bag);
    args.traj = get_arg(argc, argv, "--traj", args.traj);
    args.out = get_arg(argc, argv, "--out", args.out);
    args.depth_topic = get_arg(argc, argv, "--depth-topic", args.depth_topic);
    args.camera_info_topic = get_arg(argc, argv, "--camera-info-topic", args.camera_info_topic);
    args.height_axis = get_arg(argc, argv, "--height-axis", args.height_axis);

    std::string value;
    value = get_arg(argc, argv, "--traj-time-scale");
    if(!value.empty())
        args.traj_time_scale = std::stod(value);
    value = get_arg(argc, argv, "--traj-time-offset");
    if(!value.empty())
        args.traj_time_offset = std::stod(value);
    value = get_arg(argc, argv, "--max-pose-dt");
    if(!value.empty())
        args.max_pose_dt = std::stod(value);
    value = get_arg(argc, argv, "--depth-scale");
    if(!value.empty())
        args.depth_scale = std::stod(value);
    value = get_arg(argc, argv, "--fuse-near");
    if(!value.empty())
        args.fuse_near = std::stod(value);
    value = get_arg(argc, argv, "--fuse-far");
    if(!value.empty())
        args.fuse_far = std::stod(value);
    value = get_arg(argc, argv, "--depth-trunc");
    if(!value.empty())
        args.fuse_far = std::stod(value);
    value = get_arg(argc, argv, "--frame-stride");
    if(!value.empty())
        args.frame_stride = std::stoi(value);
    value = get_arg(argc, argv, "--max-frames");
    if(!value.empty())
        args.max_frames = std::stoi(value);
    value = get_arg(argc, argv, "--min-valid-depth-pixels");
    if(!value.empty())
        args.min_valid_depth_pixels = std::stoi(value);
    value = get_arg(argc, argv, "--min-update-times");
    if(!value.empty())
        args.min_update_times = std::stoi(value);
    parse_optional_double(argc, argv, "--start-time", args.start_time);
    parse_optional_double(argc, argv, "--end-time", args.end_time);
    parse_optional_double(argc, argv, "--skip-first-sec", args.skip_first_sec);
    parse_optional_double(argc, argv, "--skip-last-sec", args.skip_last_sec);
    parse_optional_double(argc, argv, "--min-camera-z", args.min_camera_z);
    parse_optional_double(argc, argv, "--max-camera-z", args.max_camera_z);
    parse_optional_double(argc, argv, "--max-angular-rate", args.max_angular_rate);
    args.vins_config_yaml = get_arg(argc, argv, "--vins-config-yaml", args.vins_config_yaml);
    parse_optional_double(argc, argv, "--stereo-depth-tx", args.stereo_depth_tx);
    parse_optional_double(argc, argv, "--stereo-depth-ty", args.stereo_depth_ty);
    parse_optional_double(argc, argv, "--stereo-depth-tz", args.stereo_depth_tz);
    parse_optional_double(argc, argv, "--trajectory-keep-radius", args.trajectory_keep_radius);
    parse_optional_double(argc, argv, "--density-voxel-size", args.density_voxel_size);
    value = get_arg(argc, argv, "--density-min-neighbors");
    if(!value.empty())
        args.density_min_neighbors = std::stoi(value);
    if(has_arg(argc, argv, "--keep-low-height-outliers"))
        args.remove_low_height_outliers = false;
    if(has_arg(argc, argv, "--remove-low-height-outliers"))
        args.remove_low_height_outliers = true;
    parse_optional_double(argc, argv, "--low-height-percentile", args.low_height_percentile);
    parse_optional_double(argc, argv, "--low-height-margin", args.low_height_margin);
    double parsed_value;
    if(parse_optional_double(argc, argv, "--crop-x-min", parsed_value))
        args.crop_min(0) = static_cast<float>(parsed_value);
    if(parse_optional_double(argc, argv, "--crop-y-min", parsed_value))
        args.crop_min(1) = static_cast<float>(parsed_value);
    if(parse_optional_double(argc, argv, "--crop-z-min", parsed_value))
        args.crop_min(2) = static_cast<float>(parsed_value);
    if(parse_optional_double(argc, argv, "--crop-x-max", parsed_value))
        args.crop_max(0) = static_cast<float>(parsed_value);
    if(parse_optional_double(argc, argv, "--crop-y-max", parsed_value))
        args.crop_max(1) = static_cast<float>(parsed_value);
    if(parse_optional_double(argc, argv, "--crop-z-max", parsed_value))
        args.crop_max(2) = static_cast<float>(parsed_value);
    std::string crop_polygon = get_arg(argc, argv, "--crop-polygon");
    if(!crop_polygon.empty())
        args.crop_polygon = parse_polygon(crop_polygon);

    if(args.bag.empty() || args.traj.empty())
    {
        print_usage();
        throw std::runtime_error("--bag and --traj are required");
    }
    if(args.frame_stride < 1)
        throw std::runtime_error("--frame-stride must be >= 1");
    if(args.height_axis != "x" && args.height_axis != "y" && args.height_axis != "z")
        throw std::runtime_error("--height-axis must be x, y, or z");
    if(args.trajectory_keep_radius < 0.0)
        throw std::runtime_error("--trajectory-keep-radius must be >= 0; use 0 to disable");
    if(args.density_voxel_size < 0.0)
        throw std::runtime_error("--density-voxel-size must be >= 0; use 0 to disable");
    if(args.density_min_neighbors < 0)
        throw std::runtime_error("--density-min-neighbors must be >= 0; use 0 to disable");
    if(args.low_height_percentile < 0.0 || args.low_height_percentile > 0.5)
        throw std::runtime_error("--low-height-percentile must be in [0, 0.5]");
    if(args.low_height_margin < 0.0)
        throw std::runtime_error("--low-height-margin must be >= 0");

    args.bag = resolve_package_relative_path(args.bag);
    args.traj = resolve_package_relative_path(args.traj);
    args.out = resolve_package_relative_path(args.out);
    args.vins_config_yaml = resolve_package_relative_path(args.vins_config_yaml);

    return args;
}

static double infer_time_scale(const std::vector<double> &stamps)
{
    if(stamps.empty())
        return 1.0;
    std::vector<double> sorted = stamps;
    std::sort(sorted.begin(), sorted.end());
    double median_stamp = sorted[sorted.size() / 2];
    if(median_stamp > 1.0e17)
        return 1.0e-9;
    if(median_stamp > 1.0e14)
        return 1.0e-6;
    if(median_stamp > 1.0e11)
        return 1.0e-3;
    return 1.0;
}

static Eigen::Matrix3f quat_wxyz_to_rot(float qw, float qx, float qy, float qz)
{
    float n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if(n == 0.0f)
        throw std::runtime_error("zero quaternion in trajectory");
    qw /= n;
    qx /= n;
    qy /= n;
    qz /= n;
    Eigen::Matrix3f r;
    r << 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw),
         2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw),
         2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy);
    return r;
}

static Trajectory read_trajectory(const Args &args)
{
    std::ifstream file(args.traj);
    if(!file)
        throw std::runtime_error("failed to open trajectory: " + args.traj);

    std::vector<double> raw_times;
    std::vector<Eigen::Matrix4f> poses;
    std::string line;
    while(std::getline(file, line))
    {
        if(line.empty() || line[0] == '#')
            continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::stringstream ss(line);
        std::vector<double> values;
        double v;
        while(ss >> v)
            values.push_back(v);
        if(values.empty())
            continue;
        if(values.size() < 8)
            throw std::runtime_error("trajectory row must have at least 8 columns");

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3, 3>(0, 0) = quat_wxyz_to_rot(
            static_cast<float>(values[4]),
            static_cast<float>(values[5]),
            static_cast<float>(values[6]),
            static_cast<float>(values[7]));
        pose(0, 3) = static_cast<float>(values[1]);
        pose(1, 3) = static_cast<float>(values[2]);
        pose(2, 3) = static_cast<float>(values[3]);
        raw_times.push_back(values[0]);
        poses.push_back(pose);
    }

    if(raw_times.empty())
        throw std::runtime_error("empty trajectory");

    double scale = args.traj_time_scale > 0.0 ? args.traj_time_scale : infer_time_scale(raw_times);
    std::vector<size_t> order(raw_times.size());
    for(size_t i = 0; i < order.size(); i++)
        order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return raw_times[a] < raw_times[b]; });

    Trajectory traj;
    traj.times.reserve(raw_times.size());
    traj.poses.reserve(raw_times.size());
    for(size_t idx : order)
    {
        traj.times.push_back(raw_times[idx] * scale + args.traj_time_offset);
        traj.poses.push_back(poses[idx]);
    }

    std::cout << "trajectory timestamps scaled by " << scale << "\n";
    std::cout << "trajectory time range: " << std::fixed
              << traj.times.front() << " -> " << traj.times.back() << "\n";
    return traj;
}

static bool interpolate_pose(
    const Trajectory &traj,
    double stamp,
    double max_dt,
    Eigen::Matrix4f &pose,
    double &dt,
    double &angular_rate)
{
    auto it = std::lower_bound(traj.times.begin(), traj.times.end(), stamp);
    if(it == traj.times.begin() || it == traj.times.end())
        return false;

    int right = static_cast<int>(it - traj.times.begin());
    int left = right - 1;
    double left_dt = std::fabs(stamp - traj.times[left]);
    double right_dt = std::fabs(traj.times[right] - stamp);
    dt = std::min(left_dt, right_dt);
    if(dt > max_dt)
        return false;

    double interval = traj.times[right] - traj.times[left];
    if(interval <= 0.0)
        return false;
    double alpha = (stamp - traj.times[left]) / interval;
    alpha = std::max(0.0, std::min(1.0, alpha));

    Eigen::Vector3f t_left = traj.poses[left].block<3, 1>(0, 3);
    Eigen::Vector3f t_right = traj.poses[right].block<3, 1>(0, 3);
    Eigen::Quaternionf q_left(traj.poses[left].block<3, 3>(0, 0));
    Eigen::Quaternionf q_right(traj.poses[right].block<3, 3>(0, 0));
    q_left.normalize();
    q_right.normalize();
    angular_rate = q_left.angularDistance(q_right) / interval;

    pose = Eigen::Matrix4f::Identity();
    pose.block<3, 1>(0, 3) = (1.0f - static_cast<float>(alpha)) * t_left + static_cast<float>(alpha) * t_right;
    pose.block<3, 3>(0, 0) = q_left.slerp(static_cast<float>(alpha), q_right).normalized().toRotationMatrix();
    return true;
}

static bool nearest_pose(const Trajectory &traj, double stamp, double max_dt, Eigen::Matrix4f &pose, double &dt)
{
    auto it = std::lower_bound(traj.times.begin(), traj.times.end(), stamp);
    int best = -1;
    if(it != traj.times.end())
        best = static_cast<int>(it - traj.times.begin());
    if(it != traj.times.begin())
    {
        int prev = static_cast<int>((it - traj.times.begin()) - 1);
        if(best < 0 || std::fabs(traj.times[prev] - stamp) < std::fabs(traj.times[best] - stamp))
            best = prev;
    }
    if(best < 0)
        return false;
    dt = std::fabs(traj.times[best] - stamp);
    if(dt > max_dt)
        return false;
    pose = traj.poses[best];
    return true;
}

static sensor_msgs::CameraInfoConstPtr read_camera_info(rosbag::Bag &bag, const std::string &topic)
{
    rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{topic}));
    for(const rosbag::MessageInstance &msg : view)
    {
        sensor_msgs::CameraInfoConstPtr info = msg.instantiate<sensor_msgs::CameraInfo>();
        if(info)
            return info;
    }
    return sensor_msgs::CameraInfoConstPtr();
}

static Eigen::Matrix4d matrix_from_cv_file(const cv::FileStorage &fs, const std::string &key)
{
    cv::Mat cv_T;
    fs[key] >> cv_T;
    if(cv_T.empty() || cv_T.rows != 4 || cv_T.cols != 4)
        throw std::runtime_error("missing or invalid " + key + " in VINS config yaml");

    cv::Mat cv_T64;
    cv_T.convertTo(cv_T64, CV_64F);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for(int r = 0; r < 4; r++)
    {
        for(int c = 0; c < 4; c++)
            T(r, c) = cv_T64.at<double>(r, c);
    }
    return T;
}

static Eigen::Matrix4d make_body_depth_transform_from_stereo(
    const Args &args,
    const Eigen::Matrix4d &T_body_cam0,
    const Eigen::Matrix4d &T_body_cam1)
{
    Eigen::Matrix4d T_body_stereo_center = Eigen::Matrix4d::Identity();
    T_body_stereo_center.block<3, 3>(0, 0) = T_body_cam0.block<3, 3>(0, 0);
    T_body_stereo_center.block<3, 1>(0, 3) =
        0.5 * (T_body_cam0.block<3, 1>(0, 3) + T_body_cam1.block<3, 1>(0, 3));

    Eigen::Matrix4d T_stereo_center_depth = Eigen::Matrix4d::Identity();
    T_stereo_center_depth(0, 3) = args.stereo_depth_tx;
    T_stereo_center_depth(1, 3) = args.stereo_depth_ty;
    T_stereo_center_depth(2, 3) = args.stereo_depth_tz;

    return T_body_stereo_center * T_stereo_center_depth;
}

static Eigen::Matrix4d default_body_depth_transform(const Args &args)
{
    Eigen::Matrix4d T_body_cam0 = Eigen::Matrix4d::Identity();
    T_body_cam0 << 4.7411000202683962e-03, 2.9642562385463363e-02, 9.9954931867608332e-01, 5.9354389827203972e-02,
        -9.9875349774050504e-01, 4.9807847462912225e-02, 3.2602273262435223e-03, 2.7906395900271452e-02,
        -4.9688758504367478e-02, -9.9831883525571918e-01, 2.9841756851088252e-02, 4.3355047077579083e-02,
        0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix4d T_body_cam1 = Eigen::Matrix4d::Identity();
    T_body_cam1 << 5.0140119851134202e-03, 2.9935790598650543e-02, 9.9953924791628224e-01, 6.0217803911660484e-02,
        -9.9875731234673393e-01, 4.9713501697376761e-02, 3.5211905534726744e-03, -2.0197073911788720e-02,
        -4.9585186474814164e-02, -9.9831478812557861e-01, 3.0147853854615381e-02, 4.0710376047691219e-02,
        0.0, 0.0, 0.0, 1.0;
    return make_body_depth_transform_from_stereo(args, T_body_cam0, T_body_cam1);
}

static Eigen::Matrix4f make_body_depth_transform(const Args &args)
{
    if(args.vins_config_yaml.empty())
        return default_body_depth_transform(args).cast<float>();

    cv::FileStorage fs(args.vins_config_yaml, cv::FileStorage::READ);
    if(!fs.isOpened())
        throw std::runtime_error("failed to open VINS config yaml: " + args.vins_config_yaml);

    if(!fs["body_T_depth"].empty())
        return matrix_from_cv_file(fs, "body_T_depth").cast<float>();

    const Eigen::Matrix4d T_body_cam0 = matrix_from_cv_file(fs, "body_T_cam0");
    const Eigen::Matrix4d T_body_cam1 = matrix_from_cv_file(fs, "body_T_cam1");
    return make_body_depth_transform_from_stereo(args, T_body_cam0, T_body_cam1).cast<float>();
}

static HeightCloudFilterOptions make_height_cloud_filter_options(
    const Args &args,
    const Trajectory &traj)
{
    HeightCloudFilterOptions options;
    options.crop_min = args.crop_min;
    options.crop_max = args.crop_max;
    options.crop_polygon = args.crop_polygon;
    options.trajectory_keep_radius = static_cast<float>(args.trajectory_keep_radius);
    options.density_voxel_size = static_cast<float>(args.density_voxel_size);
    options.density_min_neighbors = args.density_min_neighbors;
    options.remove_low_height_outliers = args.remove_low_height_outliers;
    options.low_height_percentile = static_cast<float>(args.low_height_percentile);
    options.low_height_margin = static_cast<float>(args.low_height_margin);
    options.trajectory_xy.reserve(traj.poses.size());
    for(const auto &pose : traj.poses)
        options.trajectory_xy.emplace_back(pose(0, 3), pose(1, 3));
    return options;
}

static cv::Mat image_to_depth_meters(const sensor_msgs::ImageConstPtr &msg, const Args &args)
{
    cv_bridge::CvImageConstPtr input = cv_bridge::toCvShare(msg, msg->encoding);
    cv::Mat depth_m;
    if(msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || msg->encoding == "mono16")
    {
        input->image.convertTo(depth_m, CV_32FC1, 1.0 / args.depth_scale);
    }
    else if(msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        depth_m = input->image.clone();
    }
    else
    {
        throw std::runtime_error("unsupported depth encoding: " + msg->encoding);
    }

    int valid = 0;
    for(int y = 0; y < depth_m.rows; y++)
    {
        float *row = depth_m.ptr<float>(y);
        for(int x = 0; x < depth_m.cols; x++)
        {
            float &d = row[x];
            if(!std::isfinite(d) || d < args.fuse_near || d > args.fuse_far)
            {
                d = 0.0f;
            }
            else
            {
                valid++;
            }
        }
    }

    if(valid < args.min_valid_depth_pixels)
        return cv::Mat();
    return depth_m;
}

static void print_available_topics(rosbag::Bag &bag)
{
    std::cout << "available image/camera/depth topics:\n";
    rosbag::View view(bag);
    const auto connections = view.getConnections();
    for(const rosbag::ConnectionInfo *connection : connections)
    {
        const std::string &topic = connection->topic;
        if(topic.find("camera") != std::string::npos ||
           topic.find("image") != std::string::npos ||
           topic.find("depth") != std::string::npos ||
           topic.find("infra") != std::string::npos)
        {
            std::cout << "  " << topic << " [" << connection->datatype << "]\n";
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offline_surfel_from_bag", ros::init_options::AnonymousName);

    try
    {
        Args args = parse_args(argc, argv);
        Trajectory traj = read_trajectory(args);
        const Eigen::Matrix4f T_body_depth = make_body_depth_transform(args);
        std::cout << "T_stereoCenter_depth translation: ["
                  << args.stereo_depth_tx << ", "
                  << args.stereo_depth_ty << ", "
                  << args.stereo_depth_tz << "]\n";
        std::cout << "T_body_depth:\n" << T_body_depth << "\n";

        rosbag::Bag bag;
        bag.open(args.bag, rosbag::bagmode::Read);
        rosbag::View full_view(bag);
        std::cout << "bag time range: " << std::fixed
                  << full_view.getBeginTime().toSec() << " -> " << full_view.getEndTime().toSec() << "\n";
        double effective_start_time = std::max(args.start_time, full_view.getBeginTime().toSec() + args.skip_first_sec);
        double effective_end_time = std::min(args.end_time, full_view.getEndTime().toSec() - args.skip_last_sec);
        std::cout << "effective fusion time range: " << effective_start_time << " -> " << effective_end_time << "\n";

        sensor_msgs::CameraInfoConstPtr camera_info = read_camera_info(bag, args.camera_info_topic);
        if(!camera_info)
        {
            std::cout << "missing camera_info topic: " << args.camera_info_topic << "\n";
            print_available_topics(bag);
            return 1;
        }

        std::cout << "camera intrinsic:\n"
                  << camera_info->K[0] << " 0 " << camera_info->K[2] << "\n"
                  << "0 " << camera_info->K[4] << " " << camera_info->K[5] << "\n"
                  << "0 0 1\n";

        ros::NodeHandle nh("~");
        SurfelMap surfel_map(
            nh,
            static_cast<int>(camera_info->width),
            static_cast<int>(camera_info->height),
            static_cast<float>(camera_info->K[0]),
            static_cast<float>(camera_info->K[4]),
            static_cast<float>(camera_info->K[2]),
            static_cast<float>(camera_info->K[5]),
            static_cast<float>(args.fuse_far),
            static_cast<float>(args.fuse_near),
            300);

        int seen_depth = 0;
        int integrated = 0;
        int skipped_stride = 0;
        int skipped_pose = 0;
        int skipped_bad_depth = 0;
        int skipped_time = 0;
        int skipped_camera_z = 0;
        int skipped_angular_rate = 0;
        double last_pose_dt = 0.0;

        rosbag::View depth_view(bag, rosbag::TopicQuery(std::vector<std::string>{args.depth_topic}));
        if(depth_view.size() == 0)
        {
            std::cout << "missing depth topic: " << args.depth_topic << "\n";
            print_available_topics(bag);
            return 1;
        }

        for(const rosbag::MessageInstance &msg : depth_view)
        {
            sensor_msgs::ImageConstPtr depth_msg = msg.instantiate<sensor_msgs::Image>();
            if(!depth_msg)
                continue;

            seen_depth++;
            if(seen_depth % args.frame_stride != 0)
            {
                skipped_stride++;
                continue;
            }

            Eigen::Matrix4f pose;
            double pose_dt = 0.0;
            double angular_rate = 0.0;
            double stamp = depth_msg->header.stamp.toSec();
            if(stamp < effective_start_time || stamp > effective_end_time)
            {
                skipped_time++;
                continue;
            }
            if(!interpolate_pose(traj, stamp, args.max_pose_dt, pose, pose_dt, angular_rate))
            {
                skipped_pose++;
                continue;
            }
            if(angular_rate > args.max_angular_rate)
            {
                skipped_angular_rate++;
                continue;
            }
            double camera_z = pose(2, 3);
            if(camera_z < args.min_camera_z || camera_z > args.max_camera_z)
            {
                skipped_camera_z++;
                continue;
            }

            cv::Mat depth_m = image_to_depth_meters(depth_msg, args);
            if(depth_m.empty())
            {
                skipped_bad_depth++;
                continue;
            }

            Eigen::Matrix4f T_world_depth = pose * T_body_depth;
            surfel_map.process_offline_frame(depth_msg->header.stamp, depth_m, T_world_depth);
            integrated++;
            last_pose_dt = pose_dt;

            if(integrated % 50 == 0)
            {
                std::cout << "integrated=" << integrated
                          << ", skipped_pose=" << skipped_pose
                          << ", skipped_bad_depth=" << skipped_bad_depth
                          << ", skipped_stride=" << skipped_stride
                          << ", skipped_time=" << skipped_time
                          << ", skipped_camera_z=" << skipped_camera_z
                          << ", skipped_angular_rate=" << skipped_angular_rate
                          << ", last_pose_dt=" << last_pose_dt << "s\n";
            }
            if(args.max_frames > 0 && integrated >= args.max_frames)
                break;
        }

        std::cout << "integrated=" << integrated
                  << ", skipped_pose=" << skipped_pose
                  << ", skipped_bad_depth=" << skipped_bad_depth
                  << ", skipped_stride=" << skipped_stride
                  << ", skipped_time=" << skipped_time
                  << ", skipped_camera_z=" << skipped_camera_z
                  << ", skipped_angular_rate=" << skipped_angular_rate
                  << ", seen_depth=" << seen_depth << "\n";

        HeightCloudFilterOptions filter_options = make_height_cloud_filter_options(args, traj);
        surfel_map.save_height_cloud(
            args.out,
            args.height_axis,
            args.min_update_times,
            filter_options);
        bag.close();
        if(args.shutdown_roslaunch_on_finish)
            kill(getppid(), SIGINT);
        return 0;
    }
    catch(const std::exception &e)
    {
        std::cerr << "offline_surfel_from_bag failed: " << e.what() << "\n";
        return 1;
    }
}
