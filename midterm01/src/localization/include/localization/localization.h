#ifndef LOCALIZATION_LOCALIZATION_H_
#define LOCALIZATION_LOCALIZATION_H_

#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "localization/imu_processor.h"
#include "localization/initial_pose_search.h"
#include "localization/road_heading_prior.h"
#include "localization/gnss_prior.h"
#include "localization/trail_heading.h"
#include "localization/map_manager.h"
#include "localization/measurement_fusion.h"
#include "localization/ndt_matcher.h"
#include "localization/state_estimator.h"
#include "localization/yaw_bank.h"

namespace localization {

struct PlyFrameInfo {
    std::int64_t timestamp_ns = 0;
    std::string path;
};

// 主節點只做 I/O、流程調度與 publish；演算法在 ImuProcessor / MapManager / NdtMatcher / StateEstimator。
class LocalizationApp {
public:
    int Start(int argc, char** argv);

private:
    // --- 啟動與資料載入 ---
    bool LoadParameters();           // 宣告並讀取所有 ROS 參數（yaml 或 -p 覆寫）
    void SetupRosInterfaces();       // 建立 RViz 用的 publisher 與 tf
    bool LoadMap();                  // 載入地圖（優先用降採樣快取），必要時體素降採樣
    bool IndexPlyFiles(const std::string& directory_path);  // 列出 PLY 並依檔名時間戳排序
    bool LoadPlyFrame(const PlyFrameInfo& frame, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& color_cloud) const;
    bool LoadInitialPose(Eigen::Matrix4f& pose) const;  // 讀 init_guess.txt（4x4 T_map_body）

    // --- 前處理：縮放（點雲沒有絕對尺度）、去非有限點與遠點、體素降採樣 ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr ScalePlyPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, double scale) const;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ScalePlyColorCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input, double scale) const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr FilterInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input) const;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr FilterColorCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input) const;

    // 主流程：逐幀 IMU 預測 → 多起點 NDT → 軟加權融合 → EKF 更新（+ 先驗量測）→ 寫出位姿
    bool RunLocalization();

    void PublishFrame(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input,
                      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& color,
                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& submap,
                      const Eigen::Matrix4f& pose, std::int64_t timestamp_ns);
    void WritePose(const Eigen::Matrix4f& pose);

    geometry_msgs::msg::TransformStamped ToTransformStamped(const Eigen::Matrix4f& matrix,
                                                            const std::string& parent_frame,
                                                            const std::string& child_frame,
                                                            std::int64_t timestamp_ns) const;
    geometry_msgs::msg::PoseStamped ToPoseStamped(const Eigen::Matrix4f& matrix, std::int64_t timestamp_ns) const;
    bool IsFinite(const Eigen::Matrix4f& matrix) const;
    bool TimestampNsFromPlyFilename(const boost::filesystem::path& file, std::int64_t& timestamp_ns);
    void Shutdown();

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_;

    std::string map_path_;
    std::string map_file_;
    std::string map_cache_file_;
    std::string bag_path_;
    std::string imu_topic_;
    std::string ply_path_;
    std::string init_guess_file_;
    std::string save_path_;
    std::string save_file_;

    double map_voxel_leaf_size_ = 1.0;
    double input_voxel_leaf_size_ = 0.5;
    int min_input_points_ = 10;
    double ply_scale_ = 8.0;

    // 前處理：影像點雲的深度誤差隨距離快速放大；0 = 不限。Track1 實測 20 m 會發散、30 m 小幅改善
    double input_max_range_ = 0.0;  // m，水平距離

    MapManager::Config map_config_;
    NdtMatcher::Config ndt_config_;
    InitialPoseSearch::Config init_search_config_;
    // GNSS 先驗（官方文件明文允許使用 GNSS）：Track1／2 在 yaml 啟用；Track3 的 bag 沒有 GNSS，不啟用。只用時間戳 <= 影格時間的樣本（causal）
    struct GnssConfig {
        bool enabled = false;
        bool use_xy = true;
        bool use_yaw = true;
        double sigma_xy = 0.7;        // m
        double sigma_yaw_deg = 2.0;
        double max_age_ms = 600.0;    // 樣本比影格時間舊超過這麼多就丟棄
    } gnss_config_;
    std::string gnss_topic_ = "/localizer_node/gnss_pose";
    TrailHeading::Config trail_config_;  // 地圖軌跡線方向當航向量測（Track3 啟用）
    RoadHeadingPrior::Config road_heading_config_;  // 第 0 幀以地圖牆面方向修正 init_guess 的 yaw（Track2 啟用）
    MeasurementFusion::Config fusion_config_;
    StateEstimator::Config estimator_config_;

    // NDT 多起點：沿預測的行進方向平移的偏移量（m），含 0；只有 0 = 單起點
    std::vector<double> multi_start_lon_offsets_{0.0};
    // 起步靜止段估計陀螺儀零偏：樣本數與標準差門檻（超過視為車已在動，改由濾波器在線估計）
    int bias_min_samples_ = 50;
    double bias_max_stddev_ = 0.003;
    // ZUPT：IMU 窗口判定靜止時釘住速度與零偏（見 StateEstimator::UpdateStationary）
    bool zupt_enabled_ = true;
    int zupt_window_samples_ = 50;
    double zupt_accel_std_max_ = 0.12;       // m/s^2，三軸標準差總和
    double zupt_accel_mean_max_ = 0.0;       // m/s^2，窗口前向加速度均值與目前加速度零偏的最大差（0 = 不檢查）
    double zupt_gyro_std_max_ = 0.009;       // rad/s，三軸標準差總和
    // 第 0 幀前靜止樣本不足時（如 Track3 只有 4 個）改用先驗：零偏是感測器常數，Track1 與 Track3 的靜止段都約 −0.004 rad/s
    double gyro_bias_prior_ = 0.0;           // rad/s
    double bias_prior_sigma_unknown_ = 0.02; // rad/s，先驗的不確定度（之後由濾波器在線估計）

    // 初始 yaw 假設銀行（車起步後才建立，見 yaw_bank.h）；offsets 是相對目前濾波器 yaw 的偏移
    bool yaw_bank_enabled_ = false;
    std::vector<double> yaw_bank_offsets_deg_{0.0};
    int yaw_bank_frames_ = 150;          // 銀行運作幀數，之後承諾分數最佳者
    double yaw_bank_start_speed_ = 0.3;  // m/s，濾波器速度超過此值（且非靜止）才起步
    std::string yaw_bank_score_ = "fitness";
    YawBank yaw_bank_;

    bool publish_frames_ = true;  // 每幀 publish 點雲、子地圖、位姿與路徑給 RViz；批次測試可關閉以省時間
    int max_frames_ = 0;          // > 0 時只處理前這麼多幀（子集測試用）
    bool keep_alive_ = true;      // 處理完後留著節點讓 RViz 看；批次測試關閉

    std::ofstream result_file_;
    std::ofstream diag_file_;
    std::ofstream bank_file_;  // yaw 假設銀行每幀每個假設的指標（離線選打分函式用）
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    std::vector<PlyFrameInfo> ply_frames_;
};

}  // namespace localization

#endif  // LOCALIZATION_LOCALIZATION_H_
