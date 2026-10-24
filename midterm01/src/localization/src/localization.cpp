#include "localization/localization.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>

#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace localization {

int LocalizationApp::Start(int argc, char** argv) {
    rclcpp::init(argc, argv);
    node_ = std::make_shared<rclcpp::Node>("image_point_cloud_localizer");

    if (!LoadParameters()) {
        Shutdown();
        return -1;
    }

    SetupRosInterfaces();

    const boost::filesystem::path output_path = boost::filesystem::path(save_path_) / save_file_;
    result_file_.open(output_path.string());
    result_file_.precision(9);
    if (!result_file_.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open result file: %s", output_path.string().c_str());
        Shutdown();
        return -1;
    }

    if (!IndexPlyFiles(ply_path_) || !LoadMap()) {
        Shutdown();
        return -1;
    }

    if (publish_global_map_) {
        PublishGlobalMap();
    }

    const bool success = RunLocalization();
    result_file_.close();

    if (success && keep_alive_ && rclcpp::ok()) {
        RCLCPP_INFO(node_->get_logger(), "Input processing finished. Keeping publishers alive for RViz (Ctrl-C to exit).");
        rclcpp::spin(node_);
    }

    Shutdown();
    return success ? 0 : -1;
}

bool LocalizationApp::LoadParameters() {
    node_->declare_parameter<std::string>("map_path", "");
    node_->declare_parameter<std::string>("map_file", "map.pcd");
    node_->declare_parameter<std::string>("map_cache_file", "");
    node_->declare_parameter<std::string>("bag_path", "");
    node_->declare_parameter<std::string>("ply_path", "");
    node_->declare_parameter<std::string>("init_guess_file", "");
    node_->declare_parameter<std::vector<double>>("initial_pose", {0.0, 0.0, 0.0, 0.0});
    node_->declare_parameter<std::string>("save_path", "");
    node_->declare_parameter<std::string>("save_file", "track_result.txt");

    node_->declare_parameter<double>("map_voxel_leaf_size", map_voxel_leaf_size_);
    node_->declare_parameter<double>("input_voxel_leaf_size", input_voxel_leaf_size_);
    node_->declare_parameter<double>("submap_half_size", submap_half_size_);
    node_->declare_parameter<double>("submap_half_height", submap_half_height_);
    node_->declare_parameter<int>("min_input_points", min_input_points_);
    node_->declare_parameter<double>("ply_scale", ply_scale_);
    
    node_->declare_parameter<double>("camera_pitch_offset", 0.0);
    node_->declare_parameter<double>("camera_yaw_offset", 0.0);
    
    node_->declare_parameter<bool>("keep_alive", keep_alive_);
    node_->declare_parameter<bool>("publish_global_map", publish_global_map_);
    node_->declare_parameter<int>("max_frames", max_frames_);

    node_->declare_parameter<double>("ndt_resolution", ndt_resolution_);
    node_->declare_parameter<double>("ndt_step_size", ndt_step_size_);
    node_->declare_parameter<double>("ndt_transformation_epsilon", ndt_transformation_epsilon_);
    node_->declare_parameter<int>("ndt_max_iterations", ndt_max_iterations_);

    node_->declare_parameter<bool>("output_raw_frame_pose", output_raw_frame_pose_);
    node_->declare_parameter<double>("max_speed", max_speed_);
    node_->declare_parameter<double>("speed_ema_alpha", speed_ema_alpha_);
    node_->declare_parameter<double>("max_long_correction", max_long_correction_);
    node_->declare_parameter<double>("max_lat_correction", max_lat_correction_);
    node_->declare_parameter<double>("max_vert_correction", max_vert_correction_);
    node_->declare_parameter<double>("max_rot_correction_deg", max_rot_correction_deg_);
    node_->declare_parameter<double>("divergence_dist", divergence_dist_);
    node_->declare_parameter<double>("divergence_angle_deg", divergence_angle_deg_);
    node_->declare_parameter<double>("max_fitness_score", max_fitness_score_);
    node_->declare_parameter<bool>("learn_forward_axis", learn_forward_axis_);
    node_->declare_parameter<double>("forward_axis_alpha", forward_axis_alpha_);
    node_->declare_parameter<double>("max_forward_axis_dev_deg", max_forward_axis_dev_deg_);

    MotionPredictorConfig mp_cfg;
    node_->declare_parameter<bool>("enable_motion_prior", mp_cfg.enable_motion_prior);
    node_->declare_parameter<double>("max_linear_velocity", mp_cfg.max_linear_velocity);
    node_->declare_parameter<double>("max_angular_velocity", mp_cfg.max_angular_velocity);
    node_->declare_parameter<double>("motion_prior_weight", mp_cfg.motion_prior_weight);
    node_->declare_parameter<double>("outlier_threshold_dist", mp_cfg.outlier_threshold_dist);

    map_path_ = node_->get_parameter("map_path").as_string();
    map_file_ = node_->get_parameter("map_file").as_string();
    map_cache_file_ = node_->get_parameter("map_cache_file").as_string();
    bag_path_ = node_->get_parameter("bag_path").as_string();
    ply_path_ = node_->get_parameter("ply_path").as_string();
    init_guess_file_ = node_->get_parameter("init_guess_file").as_string();
    initial_pose_ = node_->get_parameter("initial_pose").as_double_array();
    save_path_ = node_->get_parameter("save_path").as_string();
    save_file_ = node_->get_parameter("save_file").as_string();

    map_voxel_leaf_size_ = node_->get_parameter("map_voxel_leaf_size").as_double();
    input_voxel_leaf_size_ = node_->get_parameter("input_voxel_leaf_size").as_double();
    submap_half_size_ = node_->get_parameter("submap_half_size").as_double();
    submap_half_height_ = node_->get_parameter("submap_half_height").as_double();
    min_input_points_ = node_->get_parameter("min_input_points").as_int();
    ply_scale_ = node_->get_parameter("ply_scale").as_double();
    
    camera_pitch_offset_ = node_->get_parameter("camera_pitch_offset").as_double();
    camera_yaw_offset_ = node_->get_parameter("camera_yaw_offset").as_double();
    
    keep_alive_ = node_->get_parameter("keep_alive").as_bool();
    publish_global_map_ = node_->get_parameter("publish_global_map").as_bool();
    max_frames_ = node_->get_parameter("max_frames").as_int();

    ndt_resolution_ = node_->get_parameter("ndt_resolution").as_double();
    ndt_step_size_ = node_->get_parameter("ndt_step_size").as_double();
    ndt_transformation_epsilon_ = node_->get_parameter("ndt_transformation_epsilon").as_double();
    ndt_max_iterations_ = node_->get_parameter("ndt_max_iterations").as_int();

    output_raw_frame_pose_ = node_->get_parameter("output_raw_frame_pose").as_bool();
    max_speed_ = node_->get_parameter("max_speed").as_double();
    speed_ema_alpha_ = node_->get_parameter("speed_ema_alpha").as_double();
    max_long_correction_ = node_->get_parameter("max_long_correction").as_double();
    max_lat_correction_ = node_->get_parameter("max_lat_correction").as_double();
    max_vert_correction_ = node_->get_parameter("max_vert_correction").as_double();
    max_rot_correction_deg_ = node_->get_parameter("max_rot_correction_deg").as_double();
    divergence_dist_ = node_->get_parameter("divergence_dist").as_double();
    divergence_angle_deg_ = node_->get_parameter("divergence_angle_deg").as_double();
    max_fitness_score_ = node_->get_parameter("max_fitness_score").as_double();
    learn_forward_axis_ = node_->get_parameter("learn_forward_axis").as_bool();
    forward_axis_alpha_ = node_->get_parameter("forward_axis_alpha").as_double();
    max_forward_axis_dev_deg_ = node_->get_parameter("max_forward_axis_dev_deg").as_double();

    mp_cfg.enable_motion_prior = node_->get_parameter("enable_motion_prior").as_bool();
    mp_cfg.max_linear_velocity = node_->get_parameter("max_linear_velocity").as_double();
    mp_cfg.max_angular_velocity = node_->get_parameter("max_angular_velocity").as_double();
    mp_cfg.motion_prior_weight = node_->get_parameter("motion_prior_weight").as_double();
    mp_cfg.outlier_threshold_dist = node_->get_parameter("outlier_threshold_dist").as_double();
    motion_predictor_ = std::make_unique<MotionPredictor>(mp_cfg);

    if (ply_path_.empty() || map_path_.empty() || save_path_.empty() || save_file_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "PLY, map and output paths are required.");
        return false;
    }
    boost::filesystem::create_directories(save_path_);
    return true;
}

void LocalizationApp::SetupRosInterfaces() {
    const auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    input_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/image_point_cloud", cloud_qos);
    submap_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/image_point_cloud_submap", cloud_qos);

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/map_pc", map_qos);
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/transformed_image_point_cloud_pose", rclcpp::QoS(1));
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/localization_path", rclcpp::QoS(1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

bool LocalizationApp::LoadMap() {
    const boost::filesystem::path map_directory(map_path_);
    const boost::filesystem::path source_path = map_directory / map_file_;
    const boost::filesystem::path cache_path = map_cache_file_.empty() ? boost::filesystem::path() : map_directory / map_cache_file_;
    const bool has_cache = !cache_path.empty() && boost::filesystem::exists(cache_path);
    const boost::filesystem::path selected_path = has_cache ? cache_path : source_path;

    map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile(selected_path.string(), *map_cloud_) == -1) return false;

    if (!has_cache && map_voxel_leaf_size_ > 0.0) {
        auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(map_cloud_);
        const float leaf = static_cast<float>(map_voxel_leaf_size_);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*filtered);
        map_cloud_ = filtered;
    }
    return !map_cloud_->empty();
}

bool LocalizationApp::OpenDB3File() {
    bag_reader_.reset();
    next_db3_message_.reset();
    return true;
}

bool LocalizationApp::LoadDB3Frame(std::int64_t timestamp_ns, DB3Frame& frame) {
    (void)timestamp_ns;
    (void)frame;
    return true;
}

bool LocalizationApp::IndexPlyFiles(const std::string& directory_path) {
    ply_frames_.clear();
    const boost::filesystem::path directory(directory_path);
    if (!boost::filesystem::exists(directory) || !boost::filesystem::is_directory(directory)) return false;

    for (const auto& entry : boost::filesystem::directory_iterator(directory)) {
        if (!boost::filesystem::is_regular_file(entry.path())) continue;
        std::string extension = entry.path().extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(), [](const unsigned char value){return static_cast<char>(std::tolower(value));});
        if (extension == ".ply") {
            std::int64_t timestamp_ns = 0;
            if (TimestampNsFromPlyFilename(entry.path(), timestamp_ns)) {
                ply_frames_.push_back({timestamp_ns, entry.path().string()});
            }
        }
    }
    std::sort(ply_frames_.begin(), ply_frames_.end(), [](const PlyFrameInfo& lhs, const PlyFrameInfo& rhs){return lhs.timestamp_ns < rhs.timestamp_ns;});
    return !ply_frames_.empty();
}

bool LocalizationApp::LoadPlyFrame(const PlyFrameInfo& frame,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr& color_cloud) const {
    pcl::PCLPointCloud2 packed_cloud;
    if (pcl::io::loadPLYFile(frame.path, packed_cloud) < 0) return false;

    cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromPCLPointCloud2(packed_cloud, *cloud);
    
    color_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromPCLPointCloud2(packed_cloud, *color_cloud);
    return !cloud->empty();
}

bool LocalizationApp::LoadInitialPose(Eigen::Matrix4f& pose) const {
    pose = Eigen::Matrix4f::Identity();
    if (!init_guess_file_.empty()) {
        std::ifstream input(init_guess_file_);
        if (input.is_open()) {
            std::string line;
            for (int row = 0; row < 4; ++row) {
                std::getline(input, line);
                std::replace(line.begin(), line.end(), ',', ' ');
                std::istringstream values(line);
                for (int column = 0; column < 4; ++column) {
                    values >> pose(row, column);
                }
            }
        }
    }
    return IsFinite(pose);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LocalizationApp::ScalePlyPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, const double scale) const {
    auto scaled = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    scaled->points.reserve(input->size());
    const float factor = static_cast<float>(scale);

    const Eigen::AngleAxisf pitch_rot(static_cast<float>(camera_pitch_offset_), Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf yaw_rot(static_cast<float>(camera_yaw_offset_), Eigen::Vector3f::UnitZ());

    for (const auto& point : input->points) {
        pcl::PointXYZ output;
        Eigen::Vector3f pt(factor * point.x, factor * point.y, factor * point.z);
        if (std::abs(camera_pitch_offset_) > 1e-5) pt = pitch_rot * pt;
        if (std::abs(camera_yaw_offset_) > 1e-5) pt = yaw_rot * pt;
        
        output.x = pt.x();
        output.y = pt.y();
        output.z = pt.z();
        scaled->points.push_back(output);
    }
    scaled->width = static_cast<std::uint32_t>(scaled->size());
    scaled->height = 1;
    scaled->is_dense = input->is_dense;
    return scaled;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr LocalizationApp::ScalePlyColorCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input, const double scale) const {
    auto scaled = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    scaled->points.reserve(input->size());
    const float factor = static_cast<float>(scale);

    const Eigen::AngleAxisf pitch_rot(static_cast<float>(camera_pitch_offset_), Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf yaw_rot(static_cast<float>(camera_yaw_offset_), Eigen::Vector3f::UnitZ());

    for (const auto& point : input->points) {
        pcl::PointXYZRGB output = point;
        Eigen::Vector3f pt(factor * point.x, factor * point.y, factor * point.z);
        if (std::abs(camera_pitch_offset_) > 1e-5) pt = pitch_rot * pt;
        if (std::abs(camera_yaw_offset_) > 1e-5) pt = yaw_rot * pt;
        
        output.x = pt.x();
        output.y = pt.y();
        output.z = pt.z();
        scaled->points.push_back(output);
    }
    scaled->width = static_cast<std::uint32_t>(scaled->size());
    scaled->height = 1;
    scaled->is_dense = input->is_dense;
    return scaled;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LocalizationApp::FilterInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input) const {
    auto finite = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    finite->points.reserve(input->size());

    for (const auto& point : input->points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            finite->points.push_back(point);
        }
    }
    finite->width = static_cast<std::uint32_t>(finite->size());
    finite->height = 1;
    finite->is_dense = true;
    
    if (input_voxel_leaf_size_ <= 0.0 || finite->empty()) return finite;
    
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(finite);
    const float leaf = static_cast<float>(input_voxel_leaf_size_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*filtered);
    return filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr LocalizationApp::FilterColorCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input) const {
    auto finite = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    finite->points.reserve(input->size());
    for (const auto& point : input->points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) finite->points.push_back(point);
    }
    finite->width = static_cast<std::uint32_t>(finite->size());
    finite->height = 1;
    finite->is_dense = true;
    if (input_voxel_leaf_size_ <= 0.0 || finite->empty()) return finite;
    
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(finite);
    const float leaf = static_cast<float>(input_voxel_leaf_size_);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*filtered);
    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LocalizationApp::CropSubmap(const Eigen::Matrix4f& center_pose) const {
    auto submap = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const float x = center_pose(0, 3);
    const float y = center_pose(1, 3);
    const float z = center_pose(2, 3);
    const float half_xy = static_cast<float>(submap_half_size_);
    const float half_z = static_cast<float>(submap_half_height_);

    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(map_cloud_);
    crop.setMin(Eigen::Vector4f(x - half_xy, y - half_xy, z - half_z, 1.0f));
    crop.setMax(Eigen::Vector4f(x + half_xy, y + half_xy, z + half_z, 1.0f));
    crop.filter(*submap);
    return submap;
}

bool LocalizationApp::AlignNDT(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source,
                               const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target,
                               const Eigen::Matrix4f& initial_guess,
                               Eigen::Matrix4f& pose, double& score) const {
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setResolution(ndt_resolution_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt.setMaximumIterations(ndt_max_iterations_);
    ndt.setInputSource(source);
    ndt.setInputTarget(target);

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    ndt.align(aligned_cloud, initial_guess);

    if (ndt.hasConverged()) {
        pose = ndt.getFinalTransformation();
        score = ndt.getFitnessScore();
        return true;
    }
    return false;
}

bool LocalizationApp::RunLocalization() {
    // -------------------------------------------------------------------------
    // 座標系約定
    //   raw  ：PLY 原始座標系（乘上 ply_scale 後）
    //   body ：經 camera_pitch/yaw_offset 校正後、實際送進 NDT 的座標系
    //          p_body = T_corr * p_raw
    //   pose ：內部狀態為 body 在 map 中的位姿，map = pose * p_body
    //          → raw 的位姿 = pose * T_corr（這才是要寫進 result 的東西）
    // -------------------------------------------------------------------------
    Eigen::Matrix4f init_pose;
    if (!LoadInitialPose(init_pose)) return false;
    if (!OpenDB3File()) return false;

    const Eigen::Matrix4f T_corr = CameraCorrection();
    const Eigen::Matrix4f T_corr_inv = RigidInverse(T_corr);
    Eigen::Matrix4f pose = output_raw_frame_pose_ ? Eigen::Matrix4f(init_pose * T_corr_inv) : init_pose;

    constexpr float kDeg2Rad = static_cast<float>(M_PI) / 180.0f;
    constexpr float kRad2Deg = 180.0f / static_cast<float>(M_PI);

    // body 座標系下的上方向與前進軸（假設校正後為 x 前 / z 上）
    const Eigen::Vector3f up_b = Eigen::Vector3f::UnitZ();
    const Eigen::Vector3f fwd_b0 = Eigen::Vector3f::UnitX();
    Eigen::Vector3f fwd_b = fwd_b0;  // 會被自校正慢慢修正
    const float max_axis_dev = static_cast<float>(max_forward_axis_dev_deg_) * kDeg2Rad;

    float speed = 0.0f;
    std::int64_t prev_timestamp_ns = 0;
    std::size_t processed = 0, coasted = 0, clamped = 0;

    // 診斷用：每個 log 視窗內的橫向修正量統計（持續同號 = 被單側特徵拉扯）
    double win_lat_sum = 0.0;
    std::size_t win_lat_n = 0, win_coast = 0, win_clamp = 0;
    double win_score_sum = 0.0;

    auto orthonormalize = [](Eigen::Matrix4f& T) {
        Eigen::Quaternionf q(Eigen::Matrix3f(T.block<3, 3>(0, 0)));
        q.normalize();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
    };

    for (const auto& frame : ply_frames_) {
        if (!rclcpp::ok()) break;
        if (max_frames_ > 0 && processed >= static_cast<std::size_t>(max_frames_)) break;

        DB3Frame db3_frame;
        if (!LoadDB3Frame(frame.timestamp_ns, db3_frame)) return false;

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_color;
        if (!LoadPlyFrame(frame, raw_cloud, raw_color)) return false;

        const auto input = FilterInputCloud(ScalePlyPointCloud(raw_cloud, ply_scale_));

        float dt = 0.0f;
        bool dt_ok = false;
        if (processed > 0 && prev_timestamp_ns > 0) {
            dt = static_cast<float>((frame.timestamp_ns - prev_timestamp_ns) * 1e-9);
            dt_ok = dt > 0.001f && dt < 1.0f;
        }

        // ---------------------------------------------------------------------
        // 1. 運動先驗：沿「body 座標系中的前進軸」推進，旋轉繼承上一幀
        // ---------------------------------------------------------------------
        Eigen::Matrix4f init_guess = pose;
        if (dt_ok) {
            const float step = std::clamp(speed * dt, 0.0f, static_cast<float>(max_speed_) * dt);
            init_guess.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3) + pose.block<3, 3>(0, 0) * (fwd_b * step);
        }

        // ---------------------------------------------------------------------
        // 2. NDT
        // ---------------------------------------------------------------------
        bool ran_ndt = false, converged = false;
        double score = std::numeric_limits<double>::infinity();
        Eigen::Matrix4f candidate = init_guess;
        pcl::PointCloud<pcl::PointXYZ>::Ptr submap = CropSubmap(init_guess);
        if (static_cast<int>(input->size()) >= min_input_points_ && !submap->empty()) {
            ran_ndt = true;
            converged = AlignNDT(input, submap, init_guess, candidate, score);
        }

        // ---------------------------------------------------------------------
        // 3. 把 NDT 修正量轉到 init_guess 的車體座標系，分成 縱向/橫向/垂直/旋轉
        // ---------------------------------------------------------------------
        const Eigen::Vector3f lat_b = up_b.cross(fwd_b).normalized();  // +y = 左（x前 z上時）
        const Eigen::Vector3f lon_b = lat_b.cross(up_b).normalized();

        Eigen::Matrix4f delta = RigidInverse(init_guess) * candidate;
        const Eigen::Vector3f d_t = delta.block<3, 1>(0, 3);
        Eigen::Quaternionf dq(Eigen::Matrix3f(delta.block<3, 3>(0, 0)));
        dq.normalize();
        Eigen::AngleAxisf d_aa(dq);
        const float d_rot_deg = std::abs(d_aa.angle()) * kRad2Deg;

        const bool first = (processed == 0);
        const bool diverged = !ran_ndt || !converged || !IsFinite(candidate) ||
                              d_t.norm() > static_cast<float>(divergence_dist_) * (first ? 3.0f : 1.0f) ||
                              d_rot_deg > static_cast<float>(divergence_angle_deg_) * (first ? 2.0f : 1.0f) ||
                              score > max_fitness_score_;

        const Eigen::Matrix4f prev_pose = pose;
        float c_lat = 0.0f;

        if (diverged) {
            pose = init_guess;
            ++coasted;
            ++win_coast;
            RCLCPP_WARN(node_->get_logger(),
                        "Frame %zu: NDT rejected (conv=%d, |dt|=%.2fm, rot=%.1fdeg, score=%.3f); coasting.",
                        processed, converged, d_t.norm(), d_rot_deg, score);
        } else if (first) {
            // 第 0 幀：init_guess 帶雜訊，直接採用 NDT 結果，不限幅
            pose = candidate;
        } else {
            float c_lon = d_t.dot(lon_b);
            c_lat = d_t.dot(lat_b);
            float c_up = d_t.dot(up_b);
            const float lon_c = std::clamp(c_lon, -static_cast<float>(max_long_correction_), static_cast<float>(max_long_correction_));
            const float lat_c = std::clamp(c_lat, -static_cast<float>(max_lat_correction_), static_cast<float>(max_lat_correction_));
            const float up_c = std::clamp(c_up, -static_cast<float>(max_vert_correction_), static_cast<float>(max_vert_correction_));
            float ang = d_aa.angle();
            const float max_ang = static_cast<float>(max_rot_correction_deg_) * kDeg2Rad;
            const bool was_clamped = lon_c != c_lon || lat_c != c_lat || up_c != c_up || std::abs(ang) > max_ang;
            ang = std::clamp(ang, -max_ang, max_ang);

            // 軟限幅：不再「整幀丟棄」，而是只採納合理範圍內的修正
            // → 即使先前累積了偏差，也能每幀一點點拉回來，不會鎖死在錯誤側
            Eigen::Matrix4f delta_c = Eigen::Matrix4f::Identity();
            delta_c.block<3, 3>(0, 0) = Eigen::AngleAxisf(ang, d_aa.axis()).toRotationMatrix();
            delta_c.block<3, 1>(0, 3) = lon_b * lon_c + lat_b * lat_c + up_b * up_c;
            pose = init_guess * delta_c;
            if (was_clamped) { ++clamped; ++win_clamp; }
            c_lat = lat_c;
        }
        orthonormalize(pose);

        // ---------------------------------------------------------------------
        // 4. 只在 NDT 被採納時更新速度與前進軸
        // ---------------------------------------------------------------------
        if (!diverged && dt_ok) {
            const Eigen::Vector3f disp_b =
                prev_pose.block<3, 3>(0, 0).transpose() * (pose.block<3, 1>(0, 3) - prev_pose.block<3, 1>(0, 3));
            const float inst_speed = std::clamp(disp_b.dot(lon_b) / dt, 0.0f, static_cast<float>(max_speed_));
            const float a = static_cast<float>(speed_ema_alpha_);
            speed = (1.0f - a) * speed + a * inst_speed;

            if (learn_forward_axis_ && disp_b.norm() > 0.1f) {
                Eigen::Vector3f f = ((1.0f - static_cast<float>(forward_axis_alpha_)) * fwd_b +
                                     static_cast<float>(forward_axis_alpha_) * disp_b.normalized()).normalized();
                // 限制與初始前進軸的夾角，避免被持續偏置拉走
                const float cosang = std::clamp(f.dot(fwd_b0), -1.0f, 1.0f);
                if (std::acos(cosang) > max_axis_dev) {
                    Eigen::Vector3f perp = f - fwd_b0 * cosang;
                    if (perp.norm() > 1e-6f) {
                        perp.normalize();
                        f = std::cos(max_axis_dev) * fwd_b0 + std::sin(max_axis_dev) * perp;
                    } else {
                        f = fwd_b0;
                    }
                }
                fwd_b = f;
            }
            win_lat_sum += c_lat;
            ++win_lat_n;
            win_score_sum += score;
        }

        prev_timestamp_ns = frame.timestamp_ns;

        const auto color = FilterColorCloud(ScalePlyColorCloud(raw_color, ply_scale_));
        PublishFrame(input, color, submap, pose, frame.timestamp_ns);  // 視覺化：body 點雲配 body 位姿
        WritePose(output_raw_frame_pose_ ? Eigen::Matrix4f(pose * T_corr) : pose);
        ++processed;
        rclcpp::spin_some(node_);

        if (processed == 1 || processed % 20 == 0) {
            const double mean_lat_cm = win_lat_n ? 100.0 * win_lat_sum / win_lat_n : 0.0;
            const double mean_score = win_lat_n ? win_score_sum / win_lat_n : 0.0;
            const float axis_yaw_deg = std::atan2(fwd_b.y(), fwd_b.x()) * kRad2Deg;
            RCLCPP_INFO(node_->get_logger(),
                        "Frame %zu/%zu: speed=%.2fm/s pose=(%.2f, %.2f, %.2f) | win: lat_corr=%+.1fcm "
                        "score=%.3f coast=%zu clamp=%zu | fwd_axis_yaw=%+.2fdeg",
                        processed, ply_frames_.size(), speed, pose(0, 3), pose(1, 3), pose(2, 3),
                        mean_lat_cm, mean_score, win_coast, win_clamp, axis_yaw_deg);
            win_lat_sum = 0.0; win_lat_n = 0; win_coast = 0; win_clamp = 0; win_score_sum = 0.0;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "Finished: %zu poses, %zu coasted, %zu clamped.", processed, coasted, clamped);
    return processed > 0;
}

void LocalizationApp::PublishGlobalMap() {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*map_cloud_, message);
    message.header.frame_id = "map";
    message.header.stamp = node_->now();
    map_pub_->publish(message);
}

void LocalizationApp::PublishFrame(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& color,
                                   const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& submap,
                                   const Eigen::Matrix4f& pose, const std::int64_t timestamp_ns) {
    const rclcpp::Time recorded_time(timestamp_ns);
    sensor_msgs::msg::PointCloud2 input_message;
    if (color && !color->empty()) {
        pcl::toROSMsg(*color, input_message);
    } else {
        pcl::toROSMsg(*input, input_message);
    }
    input_message.header.frame_id = "image_point_cloud";
    input_message.header.stamp = recorded_time;
    input_cloud_pub_->publish(input_message);

    static int submap_tick = 0;
    if (++submap_tick % 10 == 0) {
        sensor_msgs::msg::PointCloud2 submap_message;
        pcl::toROSMsg(*submap, submap_message);
        submap_message.header.frame_id = "map";
        submap_message.header.stamp = recorded_time;
        submap_pub_->publish(submap_message);
    }
    tf_broadcaster_->sendTransform(ToTransformStamped(pose, "map", "image_point_cloud", timestamp_ns));
    const auto pose_message = ToPoseStamped(pose, timestamp_ns);
    pose_pub_->publish(pose_message);
    path_.header = pose_message.header;
    path_.poses.push_back(pose_message);
    path_pub_->publish(path_);
}

void LocalizationApp::WritePose(const Eigen::Matrix4f& pose) {
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 4; ++column) {
            result_file_ << pose(row, column);
            if (row == 2 && column == 3) result_file_ << '\n';
            else result_file_ << ' ';
        }
    }
    result_file_.flush();
}

geometry_msgs::msg::TransformStamped LocalizationApp::ToTransformStamped(const Eigen::Matrix4f& matrix,
                                                                         const std::string& parent_frame,
                                                                         const std::string& child_frame,
                                                                         const std::int64_t timestamp_ns) const {
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = rclcpp::Time(timestamp_ns);
    message.header.frame_id = parent_frame;
    message.child_frame_id = child_frame;
    message.transform.translation.x = matrix(0, 3);
    message.transform.translation.y = matrix(1, 3);
    message.transform.translation.z = matrix(2, 3);
    tf2::Matrix3x3 rotation(matrix(0, 0), matrix(0, 1), matrix(0, 2),
                            matrix(1, 0), matrix(1, 1), matrix(1, 2),
                            matrix(2, 0), matrix(2, 1), matrix(2, 2));
    tf2::Quaternion quaternion;
    rotation.getRotation(quaternion);
    quaternion.normalize();
    message.transform.rotation.x = quaternion.x();
    message.transform.rotation.y = quaternion.y();
    message.transform.rotation.z = quaternion.z();
    message.transform.rotation.w = quaternion.w();
    return message;
}

geometry_msgs::msg::PoseStamped LocalizationApp::ToPoseStamped(const Eigen::Matrix4f& matrix,  const std::int64_t timestamp_ns) const {
    const auto transform = ToTransformStamped(matrix, "map", "image_point_cloud", timestamp_ns);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return pose;
}

bool LocalizationApp::IsFinite(const Eigen::Matrix4f& matrix) const {
    return matrix.array().isFinite().all();
}

// 與 ScalePlyPointCloud 完全一致：p_body = R_yaw * R_pitch * p_raw
Eigen::Matrix4f LocalizationApp::CameraCorrection() const {
    const Eigen::AngleAxisf pitch_rot(static_cast<float>(camera_pitch_offset_), Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf yaw_rot(static_cast<float>(camera_yaw_offset_), Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = (yaw_rot * pitch_rot).toRotationMatrix();
    return T;
}

Eigen::Matrix4f LocalizationApp::RigidInverse(const Eigen::Matrix4f& T) {
    Eigen::Matrix4f inv = Eigen::Matrix4f::Identity();
    const Eigen::Matrix3f Rt = T.block<3, 3>(0, 0).transpose();
    inv.block<3, 3>(0, 0) = Rt;
    inv.block<3, 1>(0, 3) = -Rt * T.block<3, 1>(0, 3);
    return inv;
}

bool LocalizationApp::TimestampNsFromPlyFilename(const boost::filesystem::path& file, std::int64_t& timestamp_ns) {
    const std::string stem = file.stem().string();
    const std::string marker = "_ts_";
    const std::size_t marker_position = stem.rfind(marker);
    if (marker_position == std::string::npos) return false;
    try {
        const std::string nanoseconds = stem.substr(marker_position + marker.size());
        std::size_t parsed = 0;
        const auto value = std::stoll(nanoseconds, &parsed);
        if (parsed != nanoseconds.size()) return false;
        timestamp_ns = value;
        return timestamp_ns > 0;
    } catch (const std::exception&) {
        return false;
    }
}

void LocalizationApp::Shutdown() {
    if (result_file_.is_open()) result_file_.close();
    tf_broadcaster_.reset();
    input_cloud_pub_.reset();
    submap_pub_.reset();
    map_pub_.reset();
    pose_pub_.reset();
    path_pub_.reset();
    node_.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
}
}  // namespace localization

int main(int argc, char** argv) {
    localization::LocalizationApp app;
    return app.Start(argc, argv);
}