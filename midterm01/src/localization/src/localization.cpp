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

// ** ================================================================== **
// ** ================ Data loading & Parameter setting ================ **
// ** ================================================================== **
bool LocalizationApp::LoadParameters() {
    // 1. I/O 與系統參數宣告
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
    
    node_->declare_parameter<bool>("keep_alive", keep_alive_);
    node_->declare_parameter<bool>("publish_global_map", publish_global_map_);
    node_->declare_parameter<int>("max_frames", max_frames_);


    // 2. Coarse-to-Fine G-ICP 參數宣告
    CoarseToFineConfig gicp_cfg;
    node_->declare_parameter<double>("coarse_voxel_size", gicp_cfg.coarse.voxel_size);
    node_->declare_parameter<double>("coarse_max_corr_dist", gicp_cfg.coarse.max_correspondence_distance);
    node_->declare_parameter<int>("coarse_max_iterations", gicp_cfg.coarse.max_iterations);
    node_->declare_parameter<double>("fine_voxel_size", gicp_cfg.fine.voxel_size);
    node_->declare_parameter<double>("fine_max_corr_dist", gicp_cfg.fine.max_correspondence_distance);
    node_->declare_parameter<int>("fine_max_iterations", gicp_cfg.fine.max_iterations);
    node_->declare_parameter<double>("gicp_transformation_epsilon", gicp_cfg.transformation_epsilon);
    node_->declare_parameter<double>("gicp_rotation_epsilon", gicp_cfg.rotation_epsilon);
    node_->declare_parameter<double>("gicp_max_fitness_score", gicp_cfg.max_acceptable_score);

    // 3. Motion Predictor 參數宣告
    MotionPredictorConfig mp_cfg;
    node_->declare_parameter<bool>("enable_motion_prior", mp_cfg.enable_motion_prior);
    node_->declare_parameter<double>("max_linear_velocity", mp_cfg.max_linear_velocity);
    node_->declare_parameter<double>("max_angular_velocity", mp_cfg.max_angular_velocity);
    node_->declare_parameter<double>("motion_prior_weight", mp_cfg.motion_prior_weight);
    node_->declare_parameter<double>("outlier_threshold_dist", mp_cfg.outlier_threshold_dist);

    // 讀取參數
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
    
    keep_alive_ = node_->get_parameter("keep_alive").as_bool();
    publish_global_map_ = node_->get_parameter("publish_global_map").as_bool();
    max_frames_ = node_->get_parameter("max_frames").as_int();

    // 建構 CoarseToFineGICP 執行個體
    gicp_cfg.coarse.voxel_size = node_->get_parameter("coarse_voxel_size").as_double();
    gicp_cfg.coarse.max_correspondence_distance = node_->get_parameter("coarse_max_corr_dist").as_double();
    gicp_cfg.coarse.max_iterations = node_->get_parameter("coarse_max_iterations").as_int();
    gicp_cfg.fine.voxel_size = node_->get_parameter("fine_voxel_size").as_double();
    gicp_cfg.fine.max_correspondence_distance = node_->get_parameter("fine_max_corr_dist").as_double();
    gicp_cfg.fine.max_iterations = node_->get_parameter("fine_max_iterations").as_int();
    gicp_cfg.transformation_epsilon = node_->get_parameter("gicp_transformation_epsilon").as_double();
    gicp_cfg.rotation_epsilon = node_->get_parameter("gicp_rotation_epsilon").as_double();
    gicp_cfg.max_acceptable_score = node_->get_parameter("gicp_max_fitness_score").as_double();
    gicp_matcher_ = std::make_unique<CoarseToFineGICP>(gicp_cfg);

    // 建構 MotionPredictor 執行個體
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
    const auto cloud_qos =rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
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
    RCLCPP_INFO(node_->get_logger(), "Loading map: %s", selected_path.string().c_str());
    if (pcl::io::loadPCDFile(selected_path.string(), *map_cloud_) == -1) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to read map: %s", selected_path.string().c_str());
        return false;
    }

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
    const std::string gnss_topic = "/localizer_node/gnss_pose";
    const std::string imu_topic = "/imu/data";
    if (!bag_path_.empty()) {
        bag_reader_ = std::make_unique<rosbag2_cpp::Reader>();
        try {
            rosbag2_storage::StorageOptions options;
            options.uri = bag_path_;
            options.storage_id = "sqlite3";
            rosbag2_cpp::ConverterOptions converter;
            converter.input_serialization_format = "cdr";
            converter.output_serialization_format = "cdr";
            bag_reader_->open(options, converter);
            rosbag2_storage::StorageFilter filter;
            filter.topics = {gnss_topic, imu_topic};
            bag_reader_->set_filter(filter);
            if (bag_reader_->has_next()) next_db3_message_ = bag_reader_->read_next();
        } catch (const std::exception& error) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open sensor bag: %s", error.what());
            return false;
        }
    }
    
    return true;
}

bool LocalizationApp::LoadDB3Frame(std::int64_t timestamp_ns, DB3Frame& frame) {
    const std::string gnss_topic = "/localizer_node/gnss_pose";
    const std::string imu_topic = "/imu/data";
    rclcpp::Serialization<geometry_msgs::msg::PoseStamped> gnss_serialization;
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
    try {
        while (next_db3_message_ && next_db3_message_->recv_timestamp <= timestamp_ns) {
            rclcpp::SerializedMessage serialized(*next_db3_message_->serialized_data);
            if (next_db3_message_->topic_name == gnss_topic) {
                gnss_serialization.deserialize_message(&serialized, &frame.gnss_pose);
                frame.has_gnss_pose = true;
            } else if (next_db3_message_->topic_name == imu_topic) {
                imu_serialization.deserialize_message(&serialized, &frame.imu_data);
                frame.has_imu_data = true;
            }
            next_db3_message_ = bag_reader_->has_next() ? bag_reader_->read_next() : nullptr;
        }
    } catch (const std::exception& error) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to read sensor message: %s", error.what());
        return false;
    }
    return true;
}

bool LocalizationApp::IndexPlyFiles(const std::string& directory_path) {
    ply_frames_.clear();
    const boost::filesystem::path directory(directory_path);
    if (!boost::filesystem::exists(directory) || !boost::filesystem::is_directory(directory)) {
        RCLCPP_ERROR(node_->get_logger(), "Image PLY directory does not exist: %s", directory.string().c_str());
        return false;
    }

    for (const auto& entry : boost::filesystem::directory_iterator(directory)) {
        if (!boost::filesystem::is_regular_file(entry.path())) continue;
        std::string extension = entry.path().extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(), [](const unsigned char value) 
                       {return static_cast<char>(std::tolower(value));});
        if (extension == ".ply") {
            std::int64_t timestamp_ns = 0;
            if (!TimestampNsFromPlyFilename(entry.path(), timestamp_ns)) {
                RCLCPP_WARN(node_->get_logger(), "Ignoring PLY without a valid _ts_<nanoseconds> timestamp: %s", entry.path().string().c_str());
                continue;
            }
            ply_frames_.push_back({timestamp_ns, entry.path().string()});
        }
    }
    std::sort(ply_frames_.begin(), ply_frames_.end(), [](const PlyFrameInfo& lhs, const PlyFrameInfo& rhs) 
              {return lhs.timestamp_ns < rhs.timestamp_ns;});
              
    if (ply_frames_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "No timestamped image point-cloud .ply files found in: %s", directory.string().c_str());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Indexed %zu PLY filenames without loading point data (first=%lld, last=%lld).",
                ply_frames_.size(),
                static_cast<long long>(ply_frames_.front().timestamp_ns),
                static_cast<long long>(ply_frames_.back().timestamp_ns));
    return true;
}

bool LocalizationApp::LoadPlyFrame(const PlyFrameInfo& frame,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr& color_cloud) const {
    const boost::filesystem::path file(frame.path);
    if (!boost::filesystem::exists(file) || !boost::filesystem::is_regular_file(file) || boost::filesystem::file_size(file) == 0) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid image PLY file: %s", frame.path.c_str());
        return false;
    }

    pcl::PCLPointCloud2 packed_cloud;
    if (pcl::io::loadPLYFile(frame.path, packed_cloud) < 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to read image PLY file: %s", frame.path.c_str());
        return false;
    }

    cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromPCLPointCloud2(packed_cloud, *cloud);
    if (cloud->empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Image PLY contains no points: %s", frame.path.c_str());
        return false;
    }

    color_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromPCLPointCloud2(packed_cloud, *color_cloud);
    if (color_cloud->size() != cloud->size()) {
        RCLCPP_ERROR(node_->get_logger(), "Image PLY XYZ/RGB point counts differ: %s", frame.path.c_str());
        return false;
    }
    return true;
}

bool LocalizationApp::LoadInitialPose(Eigen::Matrix4f& pose) const {
    pose = Eigen::Matrix4f::Identity();
    if (init_guess_file_.empty()) {
        const float x = static_cast<float>(initial_pose_[0]);
        const float y = static_cast<float>(initial_pose_[1]);
        const float z = static_cast<float>(initial_pose_[2]);
        const float yaw = static_cast<float>(initial_pose_[3]);
        pose(0, 0) = std::cos(yaw); 
        pose(0, 1) = -std::sin(yaw);
        pose(1, 0) = std::sin(yaw); 
        pose(1, 1) = std::cos(yaw);
        pose(0, 3) = x;
        pose(1, 3) = y;
        pose(2, 3) = z;
    } else {
        std::ifstream input(init_guess_file_);
        if (!input.is_open()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open init guess: %s", init_guess_file_.c_str());
            return false;
        }

        std::string line;
        for (int row = 0; row < 4; ++row) {
            if (!std::getline(input, line)) {
                RCLCPP_ERROR(node_->get_logger(), "Init guess must contain four rows.");
                return false;
            }
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream values(line);
            for (int column = 0; column < 4; ++column) {
                if (!(values >> pose(row, column))) {
                    RCLCPP_ERROR(node_->get_logger(), "Invalid value in init guess row %d.", row + 1);
                    return false;
                }
            }
        }
    }

    if (!IsFinite(pose)) {
        RCLCPP_ERROR(node_->get_logger(), "Initial pose contains NaN or Inf.");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Initial pose: x=%.3f, y=%.3f, z=%.3f, yaw=%.6f",
                pose(0, 3), pose(1, 3), pose(2, 3), std::atan2(pose(1, 0), pose(0, 0)));
    return true;
}

// ** ================================================================== **
// ** ==================== input data/map processing =================== **
// ** ================================================================== **
pcl::PointCloud<pcl::PointXYZ>::Ptr LocalizationApp::ScalePlyPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, const double scale) const {
    auto scaled = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    scaled->points.reserve(input->size());
    const float factor = static_cast<float>(scale);
    for (const auto& point : input->points) {
        pcl::PointXYZ output;
        output.x = factor * point.x;
        output.y = factor * point.y;
        output.z = factor * point.z;
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

    for (const auto& point : input->points) {
        pcl::PointXYZRGB output = point;
        output.x = factor * point.x;
        output.y = factor * point.y;
        output.z = factor * point.z;
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
        if (pcl::isFinite(point)) finite->points.push_back(point);
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
        if (pcl::isFinite(point)) finite->points.push_back(point);
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

// ** ================================================================== **
// ** =========================== Localization ========================= **
// ** ================================================================== **
bool LocalizationApp::RunLocalization() {
    Eigen::Matrix4f pose;
    if (!LoadInitialPose(pose)) return false;
    
    if (!OpenDB3File()) return false;
    // DB3Frame db3_frame;
    
    std::size_t processed = 0;
    std::size_t failed = 0;

    motion_predictor_->Reset();
    for (const auto& frame : ply_frames_) {
        if (!rclcpp::ok()) break;
        if (max_frames_ > 0 && processed >= static_cast<std::size_t>(max_frames_)) break;
        DB3Frame db3_frame;
        if (!LoadDB3Frame(frame.timestamp_ns, db3_frame)) return false;
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_color;
        if (!LoadPlyFrame(frame, raw_cloud, raw_color)) return false;
        const auto input = FilterInputCloud(ScalePlyPointCloud(raw_cloud, ply_scale_));
        const auto submap = CropSubmap(pose);
        if (submap->empty()) {
            RCLCPP_ERROR(node_->get_logger(), "Empty submap at frame %zu.", processed);
            return false;
        }

        // 1. 運動學先驗動態初值推算 (過濾 noisy init_guess)
        Eigen::Matrix4f init_guess = (processed == 0) ? pose : 
            motion_predictor_->PredictInitialGuess(pose, frame.timestamp_ns);

        // Track01 / Track02：若有 GNSS 資料，可用於初值融合
        if (db3_frame.has_gnss_pose) {
            const float gx = static_cast<float>(db3_frame.gnss_pose.pose.position.x);
            const float gy = static_cast<float>(db3_frame.gnss_pose.pose.position.y);
            const float dist = std::hypot(gx - init_guess(0, 3), gy - init_guess(1, 3));
            if (dist < 3.0f) {
                init_guess(0, 3) = 0.8f * init_guess(0, 3) + 0.2f * gx;
                init_guess(1, 3) = 0.8f * init_guess(1, 3) + 0.2f * gy;
            }
        }

        // 2. Coarse-to-Fine G-ICP 雙層配準
        Eigen::Matrix4f candidate_pose = init_guess;
        double score = std::numeric_limits<double>::infinity();
        bool converged = gicp_matcher_->Align(input, submap, init_guess, candidate_pose, score);

        if (std::abs(candidate_pose(2, 3) - pose(2, 3)) > 0.3f) {
            candidate_pose(2, 3) = 0.7f * pose(2, 3) + 0.3f * candidate_pose(2, 3);
        }

        // 3. 姿態檢驗與安全推進
        Eigen::Matrix4f final_pose = candidate_pose;
        bool physically_valid = motion_predictor_->ValidateAndCorrect(
            candidate_pose, score, frame.timestamp_ns, final_pose);

        if (converged && physically_valid && score < 2.0) {
            pose = candidate_pose;
            motion_predictor_->UpdateState(pose, frame.timestamp_ns);
        } else {
            ++failed;
            // 【核心防凍結修復】：配準失敗時，不可原地停滯！
            // 若運動預測尚未累積有效速度 (位移極小)，直接採用 init_guess 往前推進
            const double displacement = (final_pose.block<3, 1>(0, 3) - pose.block<3, 1>(0, 3)).norm();
            if (displacement < 0.05) {
                pose = init_guess;
            } else {
                pose = final_pose;
            }
            // 失敗時不將靜止狀態更新進速度歷史，避免速度被洗成 0
            RCLCPP_WARN(node_->get_logger(), 
                        "Frame %zu: Alignment rejected (converged=%d, score=%.2f); progressing with prior/init.", 
                        processed, converged, score);
        }

        const auto color = FilterColorCloud(ScalePlyColorCloud(raw_color, ply_scale_));
        PublishFrame(input, color, submap, pose, frame.timestamp_ns);
        WritePose(pose);
        ++processed;
        rclcpp::spin_some(node_);
        if (processed == 1 || processed % 20 == 0) {
            RCLCPP_INFO(node_->get_logger(), "Frame %zu/%zu: G-ICP=%s, score=%.4f, pose=(%.3f, %.3f, %.3f)",
                        processed, ply_frames_.size(), converged ? "OK" : "REJECTED", score,
                        pose(0, 3), pose(1, 3), pose(2, 3));
        }
    }
    RCLCPP_INFO(node_->get_logger(), "Finished: %zu poses, %zu corrected frames.", processed, failed);
    return processed > 0;
}

// ** ================================================================== **
// ** ========================== Rviz $ Record ========================= **
// ** ================================================================== **
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
            if (row == 2 && column == 3) {
                result_file_ << '\n';
            } else {
                result_file_ << ' ';
            }
        }
    }
    result_file_.flush();
}

// ** ================================================================== **
// ** ======================= Messages transform ======================= **
// ** ================================================================== **
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
