#include "localization/localization.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <sstream>

#include <pcl/PCLPointCloud2.h>
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
    diag_file_.open(output_path.string() + ".diag.csv");
    diag_file_ << "idx,converged,fitness,mahalanobis_sq,inflation,speed,gyro_bias,beta_deg,imu_samples,ms,sigma_lon,sigma_lat,sigma_yaw,spread_lon,n_eff,accel_bias,pred_x,pred_y,pred_yaw,meas_x,meas_y,meas_yaw,final_fitness,yaw_minus_imu_deg,trail_delta_deg,trail_applied,side_share,side_active\n";

    if (yaw_bank_enabled_) {
        bank_file_.open(output_path.string() + ".bank.csv");
        bank_file_ << "idx,hyp,offset_deg,fitness,nis,lateral_innov,yaw_innov_deg,yaw_deg,x,y\n";
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
    diag_file_.close();

    if (success && keep_alive_ && rclcpp::ok()) {
        RCLCPP_INFO(node_->get_logger(), "Input processing finished. Keeping publishers alive for RViz (Ctrl-C to exit).");
        rclcpp::spin(node_);
    }

    Shutdown();
    return success ? 0 : -1;
}

bool LocalizationApp::LoadParameters() {
    auto declare_double = [this](const char* name, double& value) {
        node_->declare_parameter<double>(name, value);
        value = node_->get_parameter(name).as_double();
    };
    auto declare_int = [this](const char* name, int& value) {
        node_->declare_parameter<int>(name, value);
        value = static_cast<int>(node_->get_parameter(name).as_int());
    };
    auto declare_bool = [this](const char* name, bool& value) {
        node_->declare_parameter<bool>(name, value);
        value = node_->get_parameter(name).as_bool();
    };
    auto declare_string = [this](const char* name, std::string& value) {
        node_->declare_parameter<std::string>(name, value);
        value = node_->get_parameter(name).as_string();
    };

    map_file_ = "map.pcd";
    save_file_ = "track_result.txt";
    imu_topic_ = "/imu/data";
    declare_string("map_path", map_path_);
    declare_string("map_file", map_file_);
    declare_string("map_cache_file", map_cache_file_);
    declare_string("bag_path", bag_path_);
    declare_string("imu_topic", imu_topic_);
    declare_string("ply_path", ply_path_);
    declare_string("init_guess_file", init_guess_file_);
    declare_string("save_path", save_path_);
    declare_string("save_file", save_file_);

    declare_double("map_voxel_leaf_size", map_voxel_leaf_size_);
    declare_double("input_voxel_leaf_size", input_voxel_leaf_size_);
    declare_int("min_input_points", min_input_points_);
    declare_double("ply_scale", ply_scale_);
    declare_double("input_max_range", input_max_range_);

    declare_double("submap_half_size", map_config_.half_size);
    declare_double("submap_half_height", map_config_.half_height);
    declare_double("submap_recrop_distance", map_config_.recrop_distance);

    declare_double("ndt_resolution", ndt_config_.resolution);
    declare_double("ndt_step_size", ndt_config_.step_size);
    declare_double("ndt_transformation_epsilon", ndt_config_.transformation_epsilon);
    declare_int("ndt_max_iterations", ndt_config_.max_iterations);

    declare_bool("side_balance_enabled", side_config_.enabled);
    declare_double("side_balance_left_share_min", side_config_.left_share_min);
    declare_double("side_balance_lon_max", side_config_.lon_max);
    declare_double("side_balance_lat_min", side_config_.lat_min);
    declare_double("side_balance_lat_max", side_config_.lat_max);
    declare_double("side_balance_z_low", side_config_.z_low);
    declare_double("side_balance_z_high", side_config_.z_high);
    declare_int("side_balance_min_points", side_config_.min_points);
    declare_double("side_balance_yaw_sigma_deg", side_config_.yaw_sigma_deg);
    declare_bool("trail_heading_enabled", trail_config_.enabled);
    declare_double("trail_heading_z_band", trail_config_.z_band);
    declare_double("trail_heading_lon_half", trail_config_.lon_half);
    declare_double("trail_heading_lat_half", trail_config_.lat_half);
    declare_int("trail_heading_min_points", trail_config_.min_points);
    declare_double("trail_heading_max_thinness", trail_config_.max_thinness);
    declare_double("trail_heading_min_length", trail_config_.min_length);
    declare_double("trail_heading_gate_deg", trail_config_.gate_deg);
    declare_double("trail_heading_sigma_deg", trail_config_.sigma_deg);
    declare_int("trail_heading_period_frames", trail_config_.period_frames);
    declare_bool("trail_heading_when_stationary", trail_config_.when_stationary);
    declare_bool("road_heading_enabled", road_heading_config_.enabled);
    declare_double("road_heading_radius", road_heading_config_.radius);
    declare_double("road_heading_cell", road_heading_config_.cell);
    declare_double("road_heading_z_min", road_heading_config_.z_min);
    declare_double("road_heading_z_max", road_heading_config_.z_max);
    declare_double("road_heading_max_dev_deg", road_heading_config_.max_deviation_deg);

    declare_bool("init_search_enabled", init_search_config_.enabled);
    declare_bool("init_search_dump", init_search_dump_);
    declare_double("init_search_yaw_range_deg", init_search_config_.yaw_range_deg);
    declare_double("init_search_yaw_step_deg", init_search_config_.yaw_step_deg);
    node_->declare_parameter<std::vector<double>>("init_search_xy_offsets", init_search_config_.xy_offsets);
    init_search_config_.xy_offsets = node_->get_parameter("init_search_xy_offsets").as_double_array();
    if (init_search_config_.xy_offsets.empty()) init_search_config_.xy_offsets = {0.0};

    declare_int("bias_min_samples", bias_min_samples_);
    declare_double("bias_max_stddev", bias_max_stddev_);
    declare_double("gyro_bias_prior", gyro_bias_prior_);
    declare_bool("zupt_enabled", zupt_enabled_);
    declare_int("zupt_window_samples", zupt_window_samples_);
    declare_double("zupt_accel_std_max", zupt_accel_std_max_);
    declare_double("zupt_accel_mean_max", zupt_accel_mean_max_);
    declare_bool("yaw_bank_enabled", yaw_bank_enabled_);
    node_->declare_parameter<std::vector<double>>("yaw_bank_offsets_deg", yaw_bank_offsets_deg_);
    yaw_bank_offsets_deg_ = node_->get_parameter("yaw_bank_offsets_deg").as_double_array();
    if (yaw_bank_offsets_deg_.empty()) yaw_bank_offsets_deg_ = {0.0};
    declare_int("yaw_bank_frames", yaw_bank_frames_);
    declare_double("yaw_bank_start_speed", yaw_bank_start_speed_);
    declare_string("yaw_bank_score", yaw_bank_score_);
    declare_double("zupt_gyro_std_max", zupt_gyro_std_max_);
    declare_double("bias_prior_sigma_unknown", bias_prior_sigma_unknown_);

    auto& e = estimator_config_;
    declare_double("init_sigma_xy", e.init_sigma_xy);
    declare_double("init_sigma_yaw_deg", e.init_sigma_yaw_deg);
    declare_double("init_sigma_speed", e.init_sigma_speed);
    declare_double("init_sigma_bias", e.init_sigma_bias);
    declare_double("init_sigma_beta_deg", e.init_sigma_beta_deg);
    declare_double("init_sigma_accel_bias", e.init_sigma_accel_bias);
    declare_double("process_sigma_xy", e.process_sigma_xy);
    declare_double("process_sigma_yaw", e.process_sigma_yaw);
    declare_double("process_sigma_speed", e.process_sigma_speed);
    declare_double("process_sigma_bias", e.process_sigma_bias);
    declare_double("process_sigma_beta", e.process_sigma_beta);
    declare_double("process_sigma_accel_bias", e.process_sigma_accel_bias);
    declare_double("accel_gain", e.accel_gain);
    declare_double("max_speed", e.max_speed);
    declare_double("meas_sigma_lon", e.meas_sigma_lon);
    declare_double("meas_sigma_lat", e.meas_sigma_lat);
    declare_double("meas_sigma_yaw_deg", e.meas_sigma_yaw_deg);
    declare_double("spread_gain", e.spread_gain);
    declare_double("gate_chi2", e.gate_chi2);
    declare_double("zupt_sigma_speed", e.zupt_sigma_speed);
    declare_double("zupt_sigma_gyro_rate", e.zupt_sigma_gyro_rate);
    declare_double("zupt_sigma_accel_bias", e.zupt_sigma_accel_bias);
    declare_bool("zupt_freeze_yaw", e.zupt_freeze_yaw);
    declare_bool("zupt_hold", e.zupt_hold);
    declare_double("fusion_probability_power", fusion_config_.probability_power);
    declare_double("fusion_prior_sigma_lon", fusion_config_.prior_sigma_lon);
    declare_double("fusion_prior_sigma_lat", fusion_config_.prior_sigma_lat);
    declare_double("fusion_prior_sigma_yaw_deg", fusion_config_.prior_sigma_yaw_deg);
    declare_double("z_gain", e.z_gain);
    declare_double("tilt_gain", e.tilt_gain);
    node_->declare_parameter<std::vector<double>>("multi_start_lon_offsets", multi_start_lon_offsets_);
    multi_start_lon_offsets_ = node_->get_parameter("multi_start_lon_offsets").as_double_array();
    if (multi_start_lon_offsets_.empty()) multi_start_lon_offsets_ = {0.0};

    declare_bool("keep_alive", keep_alive_);
    declare_bool("publish_frames", publish_frames_);
    declare_bool("publish_global_map", publish_global_map_);
    declare_int("max_frames", max_frames_);

    if (ply_path_.empty() || map_path_.empty() || save_path_.empty() || save_file_.empty() || bag_path_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "PLY, map, bag and output paths are required.");
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
    for (const auto& point : input->points) {
        scaled->points.emplace_back(factor * point.x, factor * point.y, factor * point.z);
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
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
        // 遠點的深度誤差隨距離快速放大，對縱向約束是偏差而非資訊
        if (input_max_range_ > 0.0 && std::hypot(point.x, point.y) > input_max_range_) continue;
        finite->points.push_back(point);
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

bool LocalizationApp::RunLocalization() {
    Eigen::Matrix4f init_pose;
    if (!LoadInitialPose(init_pose)) return false;

    ImuProcessor imu;
    std::string error;
    if (!imu.LoadFromBag(bag_path_, imu_topic_, error)) {
        RCLCPP_ERROR(node_->get_logger(), "%s", error.c_str());
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Loaded %zu IMU samples.", imu.size());

    MapManager map_manager(map_cloud_, map_config_);
    NdtMatcher matcher(ndt_config_);
    StateEstimator estimator(estimator_config_);

    std::int64_t prev_timestamp_ns = 0;
    double bias_sigma = 0.0;  // 0 = 使用 estimator 預設
    std::size_t processed = 0, skipped_updates = 0, stationary_frames = 0;
    double inflation_sum = 0.0;
    // 診斷用：只靠陀螺儀（扣第 0 幀零偏）從第 0 幀位姿積分出的 yaw，用來對照最終位姿 yaw；不進濾波器
    double imu_yaw_ref = 0.0, imu_ref_bias = 0.0;
    MeasurementFusion fusion(fusion_config_);

    for (const auto& frame : ply_frames_) {
        if (!rclcpp::ok()) break;
        if (max_frames_ > 0 && processed >= static_cast<std::size_t>(max_frames_)) break;

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_color;
        if (!LoadPlyFrame(frame, raw_cloud, raw_color)) return false;
        auto input = FilterInputCloud(ScalePlyPointCloud(raw_cloud, ply_scale_));
        bool input_ok = static_cast<int>(input->size()) >= min_input_points_;

        const bool first = (processed == 0);
        const auto t0 = std::chrono::steady_clock::now();

        // 只取 <= 本幀時間戳的 IMU（causal）
        const ImuInterval imu_interval = imu.Advance(frame.timestamp_ns);

        Eigen::Matrix4f guess = init_pose;
        double yaw_rate_mean = 0.0, forward_accel_mean = 0.0;
        if (first) {
            if (input_ok) {
                // init_guess 的航向帶雜訊：以 init 與 IMU 姿態航向為中心做多假設 NDT，由地圖匹配挑初值
                map_manager.Update(init_pose.block<3, 1>(0, 3));
                matcher.SetTarget(map_manager.submap());
                if (road_heading_config_.enabled) {
                    const auto prior = RoadHeadingPrior(road_heading_config_).Estimate(*map_manager.submap(), init_pose);
                    if (prior.found) {
                        const double old_yaw = std::atan2(init_pose(1, 0), init_pose(0, 0));
                        const Eigen::Matrix3f turn =
                            Eigen::AngleAxisf(static_cast<float>(prior.yaw_rad - old_yaw), Eigen::Vector3f::UnitZ()).toRotationMatrix();
                        init_pose.block<3, 3>(0, 0) = turn * init_pose.block<3, 3>(0, 0);
                        guess = init_pose;
                    }
                    RCLCPP_INFO(node_->get_logger(), "Road heading prior: %s, wall %.1f deg, deviation from init %.1f deg (%d map points).",
                                prior.found ? "applied" : "rejected", prior.wall_deg, prior.deviation_deg, prior.points);
                }
                std::vector<double> centers{std::atan2(init_pose(1, 0), init_pose(0, 0))};
                double imu_yaw = 0.0;
                if (imu.LatestHeading(imu_yaw)) centers.push_back(imu_yaw);
                const auto searched = InitialPoseSearch(init_search_config_).Search(matcher, input, init_pose, centers);
                if (init_search_dump_) {
                    std::ofstream dump((boost::filesystem::path(save_path_) / (save_file_ + ".initsearch.csv")).string());
                    dump << "start_yaw_deg,lon,lat,yaw_deg,x,y,probability,fitness\n";
                    for (const auto& s : searched.samples) {
                        dump << s.start_yaw_deg << ',' << s.lon << ',' << s.lat << ',' << s.yaw_deg << ',' << s.x << ',' << s.y << ','
                             << s.probability << ',' << s.fitness << '\n';
                    }
                }
                if (searched.found) {
                    RCLCPP_INFO(node_->get_logger(),
                                "Initial pose search: %d starts, yaw %.1f -> %.1f deg (IMU yaw %.1f), prob %.3f fit %.3f.",
                                searched.tried, centers[0] * 180.0 / M_PI, searched.yaw_deg,
                                centers.size() > 1 ? centers[1] * 180.0 / M_PI : 0.0, searched.probability, searched.fitness);
                    init_pose = searched.pose;
                    guess = init_pose;
                }
            }
            double bias = 0.0, stddev = 0.0;
            if (imu.EstimateYawBias(frame.timestamp_ns, bias_min_samples_, bias, stddev) && stddev < bias_max_stddev_) {
                estimator.Initialize(init_pose, bias, 0.0);
                RCLCPP_INFO(node_->get_logger(), "Initial gyro bias %.5f rad/s (std %.5f).", bias, stddev);
            } else {
                bias_sigma = bias_prior_sigma_unknown_;
                estimator.Initialize(init_pose, gyro_bias_prior_, bias_sigma);
                RCLCPP_WARN(node_->get_logger(), "Not enough static IMU data for bias; using prior %.4f rad/s (sigma %.4f) and estimating online.", gyro_bias_prior_, bias_sigma);
            }
        }
        const double dt = first ? 0.0 : static_cast<double>(frame.timestamp_ns - prev_timestamp_ns) * 1e-9;
        // 診斷用：只靠陀螺儀（扣第 0 幀零偏）積分出的 yaw，對照最終位姿 yaw；不進濾波器
        if (!first && imu_interval.dt > 0.5 * dt) {
            imu_yaw_ref += imu_interval.yaw_increment - imu_ref_bias * imu_interval.dt;
        }
        prev_timestamp_ns = frame.timestamp_ns;

        // 對單一濾波器做「預測 → NDT → 更新」；yaw 假設銀行啟用時，每個假設各跑一次
        struct StepResult {
            Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f measured = Eigen::Matrix4f::Identity();
            StateEstimator::UpdateInfo info;
            bool stationary = false;
            bool converged = false;
            double fitness = -1.0, spread_lon = 0.0, n_eff = 0.0, side_share = 0.0;
            int side_active = 0;
        };
        const auto step = [&](StateEstimator& est) {
            StepResult r;
            r.guess = first ? init_pose : est.pose();
            if (!first) {
                // IMU 覆蓋不足時不信任陀螺儀積分，等同「不轉」（扣掉 bias 後的角速度為 0）
                const bool imu_ok = imu_interval.dt > 0.5 * dt;
                // 靜止時先把速度與零偏釘住再預測；加速度改用其零偏估計，避免積分出假速度
                r.stationary = zupt_enabled_ && imu_ok &&
                               imu.IsStationary(zupt_window_samples_, zupt_accel_std_max_, zupt_gyro_std_max_,
                                                yaw_rate_mean, forward_accel_mean);
                // 以穩定的小加速度起步時窗口標準差也很小，只看標準差會把起步誤判成靜止，並把真實加速度當成零偏
                if (r.stationary && zupt_accel_mean_max_ > 0.0 &&
                    std::abs(forward_accel_mean - est.accel_bias()) > zupt_accel_mean_max_) {
                    r.stationary = false;
                }
                if (r.stationary) est.UpdateStationary(yaw_rate_mean, forward_accel_mean);
                const double yaw_increment = imu_ok ? imu_interval.yaw_increment : est.bias() * dt;
                const double forward_accel = r.stationary ? est.accel_bias() : (imu_ok ? imu_interval.forward_accel : 0.0);
                est.Predict(dt, yaw_increment, forward_accel, r.stationary);
                r.guess = est.pose();
            }
            r.measured = r.guess;  // 診斷用：NDT 量測（未收斂時等於預測）

            if (map_manager.Update(r.guess.block<3, 1>(0, 3))) matcher.SetTarget(map_manager.submap());
            if (input_ok && !map_manager.submap()->empty()) {
                std::vector<Eigen::Matrix4f> starts;
                const Eigen::Vector3f forward(std::cos(static_cast<float>(est.yaw())),
                                              std::sin(static_cast<float>(est.yaw())), 0.0f);
                for (const double offset : multi_start_lon_offsets_) {
                    Eigen::Matrix4f start = r.guess;
                    start.block<3, 1>(0, 3) += static_cast<float>(offset) * forward;
                    starts.push_back(start);
                }
                const auto candidates = matcher.AlignAll(input, starts);
                r.converged = !candidates.empty();
                if (r.converged) {
                    // 軟加權合併候選，並以候選分散度當這一幀量測的不確定度
                    const auto merged = fusion.Merge(candidates, r.guess);
                    r.measured = merged.pose;
                    r.fitness = merged.fitness;
                    r.n_eff = merged.effective_candidates;
                    const Eigen::Vector2d heading(std::cos(est.yaw()), std::sin(est.yaw()));
                    r.spread_lon = std::sqrt(std::max(0.0, heading.dot(merged.spread_cov.topLeftCorner<2, 2>() * heading)));
                    if (first) {
                        // 第 0 幀：init_guess 帶雜訊，直接以 NDT 結果作為濾波器初值
                        est.Initialize(merged.pose, est.bias(), bias_sigma);
                    } else {
                        Eigen::Matrix3d extra_cov = merged.spread_cov;
                        if (side_config_.enabled && !map_manager.submap()->empty()) {
                            const auto side = SideBalance(side_config_).Estimate(*map_manager.submap(), r.guess, est.yaw());
                            r.side_share = side.share;
                            r.side_active = side.active ? 1 : 0;
                            if (side.active) {
                                const double sigma = side_config_.yaw_sigma_deg * M_PI / 180.0;
                                extra_cov(2, 2) += sigma * sigma / std::max(estimator_config_.spread_gain, 1e-9);
                            }
                        }
                        r.info = est.Update(merged.pose, extra_cov, r.stationary);
                    }
                }
            }
            if (r.stationary) est.UpdateStationary(yaw_rate_mean, forward_accel_mean);
            return r;
        };

        StepResult result;
        std::size_t shown_index = 0;
        if (yaw_bank_.active) {
            // 銀行階段：同一份 IMU／點雲，各 yaw 假設各自追蹤；累積指標，輸出目前最佳者（逐幀決定，仍 causal）
            std::vector<StepResult> results;
            for (std::size_t k = 0; k < yaw_bank_.members.size(); ++k) {
                auto& member = yaw_bank_.members[k];
                results.push_back(step(member.estimator));
                const StepResult& r = results.back();
                const double yaw_k = member.estimator.yaw();
                const double dx = r.measured(0, 3) - r.guess(0, 3), dy = r.measured(1, 3) - r.guess(1, 3);
                const double lateral = -std::sin(yaw_k) * dx + std::cos(yaw_k) * dy;
                const double yaw_innov = StateEstimator::WrapAngle(
                    std::atan2(r.measured(1, 0), r.measured(0, 0)) - std::atan2(r.guess(1, 0), r.guess(0, 0)));
                if (r.converged) {
                    member.fitness_sum += r.fitness;
                    member.nis_sum += r.info.mahalanobis_sq;
                    member.lateral_abs_sum += std::abs(lateral);
                    member.yaw_abs_sum += std::abs(yaw_innov);
                    ++member.count;
                }
                const Eigen::Matrix4f p_k = member.estimator.pose();
                bank_file_ << processed << ',' << k << ',' << member.offset_deg << ',' << r.fitness << ','
                           << r.info.mahalanobis_sq << ',' << lateral << ',' << yaw_innov * 180.0 / M_PI << ','
                           << std::atan2(p_k(1, 0), p_k(0, 0)) * 180.0 / M_PI << ',' << p_k(0, 3) << ',' << p_k(1, 3) << '\n';
            }
            shown_index = yaw_bank_.BestIndex(yaw_bank_score_);
            result = results[shown_index];
            ++yaw_bank_.frames;
        } else {
            result = step(estimator);
        }
        // 承諾：保留分數最佳的假設，丟掉其餘
        const StateEstimator* shown_ptr = yaw_bank_.active ? &yaw_bank_.members[shown_index].estimator : &estimator;
        if (yaw_bank_.active && yaw_bank_.frames >= yaw_bank_frames_) {
            RCLCPP_INFO(node_->get_logger(), "Yaw bank committed after %d frames: offset %+.1f deg (score %s).",
                        yaw_bank_.frames, yaw_bank_.members[shown_index].offset_deg, yaw_bank_score_.c_str());
            estimator = yaw_bank_.members[shown_index].estimator;
            yaw_bank_.active = false;
            yaw_bank_.done = true;
            yaw_bank_.members.clear();
            shown_ptr = &estimator;
        } else if (yaw_bank_.active) {
            shown_ptr = &yaw_bank_.members[shown_index].estimator;
        }
        const StateEstimator& shown = *shown_ptr;
        const Eigen::Matrix4f guess_out = result.guess;
        const bool converged = result.converged;
        const double fitness = result.fitness;
        const StateEstimator::UpdateInfo info = result.info;
        const Eigen::Matrix4f measured = result.measured;
        if (converged && !first) inflation_sum += info.inflation;
        if (result.stationary) ++stationary_frames;
        double trail_delta_deg = std::nan("");
        int trail_applied = 0;
        if (trail_config_.enabled && !yaw_bank_.active && !first && converged && trail_config_.period_frames > 0 &&
            processed % static_cast<std::size_t>(trail_config_.period_frames) == 0 && (!result.stationary || trail_config_.when_stationary) &&
            !map_manager.submap()->empty()) {
            const auto trail = TrailHeading(trail_config_).Estimate(*map_manager.submap(), estimator.pose(), estimator.yaw());
            if (trail.found) {
                trail_delta_deg = trail.delta_rad * 180.0 / M_PI;
                estimator.UpdateHeading(trail.delta_rad, trail_config_.sigma_deg * M_PI / 180.0);
                trail_applied = 1;
            }
        }
        if (!converged && !first) ++skipped_updates;

        // 車開始移動才建立 yaw 假設銀行：靜止時沒有視差，單幀的 NDT 對 yaw 沒有鑑別力
        if (yaw_bank_enabled_ && !yaw_bank_.active && !yaw_bank_.done && !first && !result.stationary &&
            estimator.speed() > yaw_bank_start_speed_) {
            yaw_bank_.members.clear();
            yaw_bank_.frames = 0;
            for (const double offset_deg : yaw_bank_offsets_deg_) {
                YawBank::Member member{estimator, offset_deg};
                member.estimator.RotateYaw(offset_deg * M_PI / 180.0);
                yaw_bank_.members.push_back(member);
            }
            yaw_bank_.active = true;
            RCLCPP_INFO(node_->get_logger(), "Yaw bank started at frame %zu with %zu hypotheses.", processed,
                        yaw_bank_.members.size());
        }

        const Eigen::Matrix4f pose = first && !converged ? init_pose : shown.pose();
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        if (publish_frames_) {
            const auto color = FilterColorCloud(ScalePlyColorCloud(raw_color, ply_scale_));
            PublishFrame(input, color, map_manager.submap(), pose, frame.timestamp_ns);
        }
        WritePose(pose);
        if (first) {
            imu_yaw_ref = std::atan2(pose(1, 0), pose(0, 0));
            imu_ref_bias = estimator.bias();
        }
        const double final_fitness = input_ok ? matcher.FitnessAt(input, pose) : -1.0;
        const double yaw_minus_imu = StateEstimator::WrapAngle(std::atan2(pose(1, 0), pose(0, 0)) - imu_yaw_ref);
        diag_file_ << processed << ',' << converged << ',' << fitness << ',' << info.mahalanobis_sq << ','
                   << info.inflation << ',' << shown.speed() << ',' << shown.bias() << ','
                   << shown.beta() * 180.0 / M_PI << ',' << imu_interval.sample_count << ',' << ms;
        double sigma_lon = 0.0, sigma_lat = 0.0, sigma_yaw = 0.0;
        shown.Sigma(sigma_lon, sigma_lat, sigma_yaw);
        diag_file_ << ',' << sigma_lon << ',' << sigma_lat << ',' << sigma_yaw << ',' << result.spread_lon << ',' << result.n_eff << ',' << shown.accel_bias();
        const auto yaw_of = [](const Eigen::Matrix4f& T) { return std::atan2(T(1, 0), T(0, 0)); };
        diag_file_ << ',' << guess_out(0, 3) << ',' << guess_out(1, 3) << ',' << yaw_of(guess_out) << ',' << measured(0, 3) << ','
                   << measured(1, 3) << ',' << yaw_of(measured) << ',' << final_fitness << ',' << yaw_minus_imu * 180.0 / M_PI << ',' << trail_delta_deg << ',' << trail_applied << ',' << result.side_share << ',' << result.side_active << '\n';
        ++processed;
        rclcpp::spin_some(node_);

        if (processed == 1 || processed % 100 == 0) {
            RCLCPP_INFO(node_->get_logger(),
                        "Frame %zu/%zu: v=%.2fm/s bias=%+.4f ba=%+.3f beta=%+.2fdeg pose=(%.1f, %.1f) fit=%.3f infl=%.1f ms=%.0f",
                        processed, ply_frames_.size(), shown.speed(), shown.bias(), shown.accel_bias(),
                        shown.beta() * 180.0 / M_PI, pose(0, 3), pose(1, 3), fitness, info.inflation, ms);
        }
    }
    RCLCPP_INFO(node_->get_logger(), "Finished: %zu poses, %zu frames without NDT update, %zu stationary (ZUPT) frames, mean R inflation %.2f.",
                processed, skipped_updates, stationary_frames, processed > 1 ? inflation_sum / static_cast<double>(processed - 1) : 1.0);
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
    if (diag_file_.is_open()) diag_file_.close();
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
