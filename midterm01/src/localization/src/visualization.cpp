// 離線視覺化：依 result.txt 重播定位結果。
// 每幀把對應的 PLY（乘上 ply_scale）以該幀位姿轉到 map 座標發布，並發布位姿、軌跡與 TF，
// 可選擇疊加最近 N 幀點雲以檢查與地圖的對齊程度，以及顯示參考軌跡（如 GNSS，僅供目視比對）。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "localization/ply_sequence.h"

namespace localization {
namespace {

using MapCloud = pcl::PointCloud<pcl::PointXYZI>;
using FrameCloud = pcl::PointCloud<pcl::PointXYZRGB>;

constexpr char kMapFrame[] = "map";
constexpr char kBodyFrame[] = "body";

// 4x4 位姿矩陣轉 ROS Pose
geometry_msgs::msg::Pose ToPoseMsg(const Eigen::Matrix4f& T) {
    Eigen::Quaternionf q(Eigen::Matrix3f(T.block<3, 3>(0, 0)));
    q.normalize();
    geometry_msgs::msg::Pose pose;
    pose.position.x = T(0, 3);
    pose.position.y = T(1, 3);
    pose.position.z = T(2, 3);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

class VisualizationApp {
public:
    explicit VisualizationApp(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

    // 宣告參數、建立 publisher（地圖與參考軌跡用 latched QoS，RViz 晚開也看得到）、載入輸入並發布地圖與參考軌跡
    bool Init() {
        DeclareParameters();
        auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/visual/map", latched);
        ref_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/visual/reference_path", latched);
        frame_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/visual/frame", rclcpp::QoS(1));
        accum_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/visual/accumulated", rclcpp::QoS(1));
        pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/visual/pose", rclcpp::QoS(1));
        path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/visual/path", latched);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

        if (!LoadInputs()) return false;
        PublishMap();
        PublishReferencePath();
        return true;
    }

    // 依位姿檔逐幀重播（可指定起訖幀、跳幀、循環）；播放速度依 PLY 時間戳的實際間隔
    bool Play() {
        const std::size_t begin = static_cast<std::size_t>(std::max(0, start_frame_));
        const std::size_t end = end_frame_ < 0 ? poses_.size() : std::min<std::size_t>(end_frame_ + 1, poses_.size());
        if (begin >= end) {
            RCLCPP_ERROR(node_->get_logger(), "播放範圍為空：start_frame=%d end_frame=%d（共 %zu 幀）", start_frame_,
                         end_frame_, poses_.size());
            return false;
        }
        do {
            path_.poses.clear();
            accumulated_.clear();
            for (std::size_t i = begin; i < end && rclcpp::ok(); i += frame_step_) {
                const auto tick = std::chrono::steady_clock::now();
                PublishFrame(i);
                rclcpp::spin_some(node_);
                if (i % 100 == 0) RCLCPP_INFO(node_->get_logger(), "幀 %zu / %zu", i, poses_.size());
                WaitForNextFrame(i, std::min(i + frame_step_, end - 1), tick);
            }
        } while (loop_ && rclcpp::ok());
        return true;
    }

    bool keep_alive() const { return keep_alive_; }

private:
    // 宣告並讀取所有參數（檔案路徑、ply_scale、顯示用的降採樣與裁切、播放設定）
    void DeclareParameters() {
        map_file_ = node_->declare_parameter<std::string>("map_file", "");
        pose_file_ = node_->declare_parameter<std::string>("pose_file", "");
        ply_dir_ = node_->declare_parameter<std::string>("ply_dir", "");
        reference_file_ = node_->declare_parameter<std::string>("reference_file", "");
        ply_scale_ = node_->declare_parameter<double>("ply_scale", 8.0);
        frame_voxel_ = node_->declare_parameter<double>("frame_voxel", 0.0);
        map_min_intensity_ = node_->declare_parameter<double>("map_min_intensity", -1.0);
        map_crop_margin_ = node_->declare_parameter<double>("map_crop_margin", 150.0);
        playback_rate_ = node_->declare_parameter<double>("playback_rate", 1.0);
        start_frame_ = static_cast<int>(node_->declare_parameter<int>("start_frame", 0));
        end_frame_ = static_cast<int>(node_->declare_parameter<int>("end_frame", -1));
        frame_step_ = static_cast<std::size_t>(std::max<int64_t>(1, node_->declare_parameter<int>("frame_step", 1)));
        accumulate_frames_ = static_cast<std::size_t>(std::max<int64_t>(0, node_->declare_parameter<int>("accumulate_frames", 0)));
        loop_ = node_->declare_parameter<bool>("loop", false);
        keep_alive_ = node_->declare_parameter<bool>("keep_alive", true);
    }

    // 讀位姿檔與 PLY 索引；兩者幀數不同時只播放較短的那個（提交檔必須逐幀對應）
    bool LoadInputs() {
        const auto logger = node_->get_logger();
        std::string error;
        if (!LoadPoseFile(pose_file_, poses_, error)) {
            RCLCPP_ERROR(logger, "%s", error.c_str());
            return false;
        }
        frames_ = IndexPlyDirectory(ply_dir_);
        if (poses_.empty() || frames_.empty()) {
            RCLCPP_ERROR(logger, "位姿 %zu 筆、PLY %zu 幀（%s），無法播放", poses_.size(), frames_.size(), ply_dir_.c_str());
            return false;
        }
        if (poses_.size() != frames_.size()) {
            RCLCPP_WARN(logger, "位姿數 (%zu) 與 PLY 幀數 (%zu) 不一致，只播放前 %zu 幀；提交檔必須逐幀對應", poses_.size(),
                        frames_.size(), std::min(poses_.size(), frames_.size()));
            poses_.resize(std::min(poses_.size(), frames_.size()));
        }
        RCLCPP_INFO(logger, "位姿 %zu 筆，PLY 目錄 %s", poses_.size(), ply_dir_.c_str());
        return true;
    }

    // 發布地圖（只保留軌跡附近、可選強度下限），避免 RViz 顯示整張地圖過慢
    void PublishMap() {
        const auto logger = node_->get_logger();
        if (map_file_.empty()) return;
        auto map = std::make_shared<MapCloud>();
        if (pcl::io::loadPCDFile(map_file_, *map) < 0 || map->empty()) {
            RCLCPP_WARN(logger, "讀取地圖失敗，略過地圖顯示: %s", map_file_.c_str());
            return;
        }
        const std::size_t loaded = map->size();
        // 只保留軌跡附近的地圖，避免 RViz 顯示整張地圖過慢
        if (map_crop_margin_ >= 0.0) {
            Eigen::Vector3f lo = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
            Eigen::Vector3f hi = -lo;
            for (const auto& T : poses_) {
                lo = lo.cwiseMin(T.block<3, 1>(0, 3));
                hi = hi.cwiseMax(T.block<3, 1>(0, 3));
            }
            const float m = static_cast<float>(map_crop_margin_);
            pcl::CropBox<pcl::PointXYZI> crop;
            crop.setInputCloud(map);
            crop.setMin(Eigen::Vector4f(lo.x() - m, lo.y() - m, lo.z() - m, 1.0f));
            crop.setMax(Eigen::Vector4f(hi.x() + m, hi.y() + m, hi.z() + m, 1.0f));
            auto cropped = std::make_shared<MapCloud>();
            crop.filter(*cropped);
            map = cropped;
        }
        if (map_min_intensity_ >= 0.0) {
            auto filtered = std::make_shared<MapCloud>();
            for (const auto& p : map->points) {
                if (p.intensity >= map_min_intensity_) filtered->push_back(p);
            }
            map = filtered;
        }
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*map, msg);
        msg.header.frame_id = kMapFrame;
        msg.header.stamp = node_->now();
        map_pub_->publish(msg);
        RCLCPP_INFO(logger, "地圖 %zu 點，顯示 %zu 點", loaded, map->size());
    }

    // 參考軌跡 csv：需有 x、y、z 欄位（如 tools/extract_bag.py 輸出的 gnss.csv）
    void PublishReferencePath() {
        if (reference_file_.empty()) return;
        std::ifstream in(reference_file_);
        std::string line;
        if (!in.is_open() || !std::getline(in, line)) {
            RCLCPP_WARN(node_->get_logger(), "無法讀取參考軌跡: %s", reference_file_.c_str());
            return;
        }
        std::vector<std::string> header;
        std::stringstream hs(line);
        for (std::string col; std::getline(hs, col, ',');) header.push_back(col);
        auto column = [&header](const std::string& name) {
            return static_cast<int>(std::find(header.begin(), header.end(), name) - header.begin());
        };
        const int cx = column("x"), cy = column("y"), cz = column("z");
        if (cx >= static_cast<int>(header.size()) || cy >= static_cast<int>(header.size())) {
            RCLCPP_WARN(node_->get_logger(), "參考軌跡缺少 x/y 欄位: %s", reference_file_.c_str());
            return;
        }
        nav_msgs::msg::Path ref;
        ref.header.frame_id = kMapFrame;
        ref.header.stamp = node_->now();
        while (std::getline(in, line)) {
            std::vector<double> v;
            std::stringstream ss(line);
            for (std::string cell; std::getline(ss, cell, ',');) v.push_back(cell.empty() ? 0.0 : std::stod(cell));
            if (static_cast<int>(v.size()) <= std::max(cx, cy)) continue;
            geometry_msgs::msg::PoseStamped p;
            p.header = ref.header;
            p.pose.position.x = v[cx];
            p.pose.position.y = v[cy];
            p.pose.position.z = cz < static_cast<int>(v.size()) ? v[cz] : 0.0;
            p.pose.orientation.w = 1.0;
            ref.poses.push_back(p);
        }
        ref_path_pub_->publish(ref);
        RCLCPP_INFO(node_->get_logger(), "參考軌跡 %zu 點", ref.poses.size());
    }

    // 發布一幀：tf、位姿、累積路徑，以及該幀點雲（乘上 ply_scale 後以該幀位姿轉到 map）和最近 N 幀疊加
    void PublishFrame(std::size_t i) {
        const Eigen::Matrix4f& T = poses_[i];
        const auto stamp = node_->now();

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = kMapFrame;
        tf.child_frame_id = kBodyFrame;
        const auto pose = ToPoseMsg(T);
        tf.transform.translation.x = pose.position.x;
        tf.transform.translation.y = pose.position.y;
        tf.transform.translation.z = pose.position.z;
        tf.transform.rotation = pose.orientation;
        tf_broadcaster_->sendTransform(tf);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = kMapFrame;
        pose_msg.pose = pose;
        pose_pub_->publish(pose_msg);
        path_.header = pose_msg.header;
        path_.poses.push_back(pose_msg);
        path_pub_->publish(path_);

        auto cloud = LoadFrame(frames_[i].path);
        if (!cloud) return;
        auto in_map = std::make_shared<FrameCloud>();
        pcl::transformPointCloud(*cloud, *in_map, T);
        PublishCloud(frame_pub_, *in_map, stamp);

        if (accumulate_frames_ > 0) {
            accumulated_.push_back(in_map);
            while (accumulated_.size() > accumulate_frames_) accumulated_.pop_front();
            FrameCloud merged;
            for (const auto& c : accumulated_) merged += *c;
            PublishCloud(accum_pub_, merged, stamp);
        }
    }

    // 讀一幀 PLY 並乘上 ply_scale；frame_voxel > 0 時再降採樣（只為顯示）
    FrameCloud::Ptr LoadFrame(const std::string& path) const {
        auto cloud = std::make_shared<FrameCloud>();
        if (pcl::io::loadPLYFile(path, *cloud) < 0 || cloud->empty()) {
            RCLCPP_WARN(node_->get_logger(), "讀取 PLY 失敗: %s", path.c_str());
            return nullptr;
        }
        const float s = static_cast<float>(ply_scale_);
        for (auto& p : cloud->points) {
            p.x *= s;
            p.y *= s;
            p.z *= s;
        }
        if (frame_voxel_ <= 0.0) return cloud;
        auto filtered = std::make_shared<FrameCloud>();
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        const float leaf = static_cast<float>(frame_voxel_);
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*filtered);
        return filtered;
    }

    // 發布 map 座標下的點雲
    void PublishCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub, const FrameCloud& cloud,
                      const rclcpp::Time& stamp) const {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = kMapFrame;
        msg.header.stamp = stamp;
        pub->publish(msg);
    }

    // 依 PLY 時間戳的實際間隔播放；playback_rate <= 0 時不等待
    void WaitForNextFrame(std::size_t current, std::size_t next, std::chrono::steady_clock::time_point tick) const {
        if (playback_rate_ <= 0.0 || next <= current) return;
        const double dt = (frames_[next].timestamp_ns - frames_[current].timestamp_ns) * 1e-9 / playback_rate_;
        std::this_thread::sleep_until(tick + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                 std::chrono::duration<double>(std::clamp(dt, 0.0, 1.0))));
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_, frame_pub_, accum_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, ref_path_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string map_file_, pose_file_, ply_dir_, reference_file_;
    double ply_scale_ = 8.0;
    double frame_voxel_ = 0.0;
    double map_min_intensity_ = -1.0;
    double map_crop_margin_ = 150.0;
    double playback_rate_ = 1.0;
    int start_frame_ = 0;
    int end_frame_ = -1;
    std::size_t frame_step_ = 1;
    std::size_t accumulate_frames_ = 0;
    bool loop_ = false;
    bool keep_alive_ = true;

    std::vector<Eigen::Matrix4f> poses_;
    std::vector<PlyFrame> frames_;
    nav_msgs::msg::Path path_;
    std::deque<FrameCloud::Ptr> accumulated_;
};

}  // namespace
}  // namespace localization

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("result_visualization");
    localization::VisualizationApp app(node);
    bool ok = app.Init() && app.Play();
    if (ok && app.keep_alive() && rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "播放結束，保持發布供 RViz 檢視（Ctrl-C 離開）");
        rclcpp::spin(node);
    }
    if (rclcpp::ok()) rclcpp::shutdown();
    return ok ? 0 : 1;
}
