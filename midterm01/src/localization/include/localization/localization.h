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
#include <sensor_msgs/msg/imu.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 引入額外的演算法模組標頭檔
#include "localization/motion_predictor.h"

namespace localization {

struct PlyFrameInfo {
    std::int64_t timestamp_ns = 0;
    std::string path;
};

struct DB3Frame {
    geometry_msgs::msg::PoseStamped gnss_pose;
    sensor_msgs::msg::Imu imu_data;
    bool has_gnss_pose = false;
    bool has_imu_data = false;
    std::int64_t gnss_bag_timestamp_ns = 0;
    std::int64_t imu_bag_timestamp_ns = 0;
};

class LocalizationApp {
public:
    int Start(int argc, char** argv);

private:
    bool LoadParameters();
    void SetupRosInterfaces();
    bool LoadMap();
    bool OpenDB3File();
    bool LoadDB3Frame(std::int64_t timestamp_ns, DB3Frame& frame);
    bool IndexPlyFiles(const std::string& directory_path);
    bool LoadPlyFrame(const PlyFrameInfo& frame, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& color_cloud) const;
    bool LoadInitialPose(Eigen::Matrix4f& pose) const;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr ScalePlyPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, double scale) const;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ScalePlyColorCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input, double scale) const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr FilterInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input) const;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr FilterColorCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input) const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr CropSubmap(const Eigen::Matrix4f& center_pose) const;

    bool RunLocalization();
    
    bool AlignNDT(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source,
                  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target,
                  const Eigen::Matrix4f& initial_guess,
                  Eigen::Matrix4f& pose, double& score) const;
    
    void PublishGlobalMap();
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
    Eigen::Matrix4f CameraCorrection() const;
    static Eigen::Matrix4f RigidInverse(const Eigen::Matrix4f& T);
    bool TimestampNsFromPlyFilename(const boost::filesystem::path& file, std::int64_t& timestamp_ns);
    void Shutdown();

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_;

    std::unique_ptr<rosbag2_cpp::Reader> bag_reader_;
    rosbag2_storage::SerializedBagMessageSharedPtr next_db3_message_;
    
    std::string map_path_;
    std::string map_file_;
    std::string map_cache_file_;
    std::string bag_path_;
    std::string ply_path_;
    std::string init_guess_file_;
    std::vector<double> initial_pose_;
    std::string save_path_;
    std::string save_file_;
    
    double map_voxel_leaf_size_ = 1.0;
    double input_voxel_leaf_size_ = 0.3;
    int min_input_points_ = 10;
    double submap_half_size_ = 100.0;
    double submap_half_height_ = 30.0;

    double ndt_resolution_ = 2.0;
    double ndt_step_size_ = 0.1;
    double ndt_transformation_epsilon_ = 0.01;
    int ndt_max_iterations_ = 35;
    
    double ply_scale_ = 3.0;
    double camera_pitch_offset_ = 0.0;
    double camera_yaw_offset_ = 0.0; // 新增：用於手動校正 Z 軸偏航

    // ---- 輸出座標系 ----
    // true：init_guess.txt 與輸出皆為「原始 PLY 座標系」的位姿（offset 視為外參，輸出時補回）
    // false：沿用舊行為（init 與輸出皆為校正後座標系）
    bool output_raw_frame_pose_ = true;

    // ---- 運動先驗 ----
    double max_speed_ = 25.0;              // m/s
    double speed_ema_alpha_ = 0.3;

    // ---- NDT 修正量限幅（在 init_guess 車體座標系下分解）----
    double max_long_correction_ = 0.5;     // m / frame，縱向（退化方向）
    double max_lat_correction_ = 0.3;      // m / frame，橫向
    double max_vert_correction_ = 0.2;     // m / frame，垂直
    double max_rot_correction_deg_ = 1.5;  // deg / frame
    double divergence_dist_ = 3.0;         // 超過即判定發散 → 滑行
    double divergence_angle_deg_ = 10.0;
    double max_fitness_score_ = 1e9;       // 預設不啟用，先看 log 再決定

    // ---- 前進軸自校正（取代手調 camera_yaw_offset）----
    bool learn_forward_axis_ = true;
    double forward_axis_alpha_ = 0.005;
    double max_forward_axis_dev_deg_ = 5.0;

    bool publish_global_map_ = false;
    int max_frames_ = 0;
    bool keep_alive_ = true;

    std::ofstream result_file_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    std::vector<PlyFrameInfo> ply_frames_;

    std::unique_ptr<MotionPredictor> motion_predictor_;
};

}  // namespace localization

#endif  // LOCALIZATION_LOCALIZATION_H_