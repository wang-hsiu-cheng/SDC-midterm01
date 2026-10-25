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
    diag_file_ << "idx,converged,fitness,mahalanobis_sq,inflation,speed,gyro_bias,beta_deg,imu_samples,ms,sigma_lon,sigma_lat,spread_lon,n_eff,accel_bias,pred_x,pred_y,pred_yaw,meas_x,meas_y,meas_yaw\n";

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

    declare_int("bias_min_samples", bias_min_samples_);
    declare_double("bias_max_stddev", bias_max_stddev_);
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
    declare_double("fusion_probability_power", fusion_config_.probability_power);
    declare_double("fusion_prior_sigma_lon", fusion_config_.prior_sigma_lon);
    declare_double("fusion_prior_sigma_lat", fusion_config_.prior_sigma_lat);
    declare_double("fusion_prior_sigma_yaw_deg", fusion_config_.prior_sigma_yaw_deg);
    declare_double("z_gain", e.z_gain);
    declare_double("tilt_gain", e.tilt_gain);
    declare_double("fitness_ref", fitness_ref_);
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
    std::size_t processed = 0, skipped_updates = 0;
    double inflation_sum = 0.0;
    MeasurementFusion fusion(fusion_config_);

    for (const auto& frame : ply_frames_) {
        if (!rclcpp::ok()) break;
        if (max_frames_ > 0 && processed >= static_cast<std::size_t>(max_frames_)) break;

        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_color;
        if (!LoadPlyFrame(frame, raw_cloud, raw_color)) return false;
        const auto input = FilterInputCloud(ScalePlyPointCloud(raw_cloud, ply_scale_));
        const bool input_ok = static_cast<int>(input->size()) >= min_input_points_;

        const bool first = (processed == 0);
        const auto t0 = std::chrono::steady_clock::now();

        // 只取 <= 本幀時間戳的 IMU（causal）
        const ImuInterval imu_interval = imu.Advance(frame.timestamp_ns);

        Eigen::Matrix4f guess = init_pose;
        if (first) {
            double bias = 0.0, stddev = 0.0;
            if (imu.EstimateYawBias(frame.timestamp_ns, bias_min_samples_, bias, stddev) && stddev < bias_max_stddev_) {
                estimator.Initialize(init_pose, bias, 0.0);
                RCLCPP_INFO(node_->get_logger(), "Initial gyro bias %.5f rad/s (std %.5f).", bias, stddev);
            } else {
                bias_sigma = bias_prior_sigma_unknown_;
                estimator.Initialize(init_pose, 0.0, bias_sigma);
                RCLCPP_WARN(node_->get_logger(), "Not enough static IMU data for bias; estimating online.");
            }
        } else {
            const double dt = static_cast<double>(frame.timestamp_ns - prev_timestamp_ns) * 1e-9;
            // IMU 覆蓋不足時不信任陀螺儀積分，等同「不轉」（扣掉 bias 後的角速度為 0）
            const bool imu_ok = imu_interval.dt > 0.5 * dt;
            const double yaw_increment = imu_ok ? imu_interval.yaw_increment : estimator.bias() * dt;
            estimator.Predict(dt, yaw_increment, imu_ok ? imu_interval.forward_accel : 0.0);
            guess = estimator.pose();
        }
        prev_timestamp_ns = frame.timestamp_ns;

        if (map_manager.Update(guess.block<3, 1>(0, 3))) matcher.SetTarget(map_manager.submap());

        bool converged = false;
        double fitness = -1.0;
        StateEstimator::UpdateInfo info;
        Eigen::Matrix4f measured = guess;  // 診斷用：NDT 量測（未收斂時等於預測）
        double spread_lon = 0.0, n_eff = 0.0;
        if (input_ok && !map_manager.submap()->empty()) {
            std::vector<Eigen::Matrix4f> starts;
            const Eigen::Vector3f forward(std::cos(static_cast<float>(estimator.yaw())),
                                          std::sin(static_cast<float>(estimator.yaw())), 0.0f);
            for (const double offset : multi_start_lon_offsets_) {
                Eigen::Matrix4f start = guess;
                start.block<3, 1>(0, 3) += static_cast<float>(offset) * forward;
                starts.push_back(start);
            }
            const auto candidates = matcher.AlignAll(input, starts);
            converged = !candidates.empty();
            if (converged) {
                // 軟加權合併候選，並以候選分散度當這一幀量測的不確定度
                const auto merged = fusion.Merge(candidates, guess);
                measured = merged.pose;
                fitness = merged.fitness;
                n_eff = merged.effective_candidates;
                const Eigen::Vector2d heading(std::cos(estimator.yaw()), std::sin(estimator.yaw()));
                spread_lon = std::sqrt(std::max(0.0, heading.dot(merged.spread_cov.topLeftCorner<2, 2>() * heading)));
                if (first) {
                    // 第 0 幀：init_guess 帶雜訊，直接以 NDT 結果作為濾波器初值
                    estimator.Initialize(merged.pose, estimator.bias(), bias_sigma);
                } else {
                    const double r_scale = (fitness_ref_ > 0.0 && fitness > fitness_ref_)
                                               ? std::pow(fitness / fitness_ref_, 2) : 1.0;
                    info = estimator.Update(merged.pose, r_scale, merged.spread_cov);
                    inflation_sum += info.inflation;
                }
            }
        }
        if (!converged && !first) ++skipped_updates;

        const Eigen::Matrix4f pose = first && !converged ? init_pose : estimator.pose();
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        if (publish_frames_) {
            const auto color = FilterColorCloud(ScalePlyColorCloud(raw_color, ply_scale_));
            PublishFrame(input, color, map_manager.submap(), pose, frame.timestamp_ns);
        }
        WritePose(pose);
        diag_file_ << processed << ',' << converged << ',' << fitness << ',' << info.mahalanobis_sq << ','
                   << info.inflation << ',' << estimator.speed() << ',' << estimator.bias() << ','
                   << estimator.beta() * 180.0 / M_PI << ',' << imu_interval.sample_count << ',' << ms;
        double sigma_lon = 0.0, sigma_lat = 0.0;
        estimator.PositionSigma(sigma_lon, sigma_lat);
        diag_file_ << ',' << sigma_lon << ',' << sigma_lat << ',' << spread_lon << ',' << n_eff << ',' << estimator.accel_bias();
        const auto yaw_of = [](const Eigen::Matrix4f& T) { return std::atan2(T(1, 0), T(0, 0)); };
        diag_file_ << ',' << guess(0, 3) << ',' << guess(1, 3) << ',' << yaw_of(guess) << ',' << measured(0, 3) << ','
                   << measured(1, 3) << ',' << yaw_of(measured) << '\n';
        ++processed;
        rclcpp::spin_some(node_);

        if (processed == 1 || processed % 100 == 0) {
            RCLCPP_INFO(node_->get_logger(),
                        "Frame %zu/%zu: v=%.2fm/s bias=%+.4f ba=%+.3f beta=%+.2fdeg pose=(%.1f, %.1f) fit=%.3f infl=%.1f ms=%.0f",
                        processed, ply_frames_.size(), estimator.speed(), estimator.bias(), estimator.accel_bias(),
                        estimator.beta() * 180.0 / M_PI, pose(0, 3), pose(1, 3), fitness, info.inflation, ms);
        }
    }
    RCLCPP_INFO(node_->get_logger(), "Finished: %zu poses, %zu frames without NDT update, mean R inflation %.2f.",
                processed, skipped_updates, processed > 1 ? inflation_sum / static_cast<double>(processed - 1) : 1.0);
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
