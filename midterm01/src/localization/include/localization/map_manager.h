#ifndef LOCALIZATION_MAP_MANAGER_H_
#define LOCALIZATION_MAP_MANAGER_H_

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace localization {

// 維護以車輛為中心的子地圖。每幀都對全圖 CropBox 太貴，而且 NDT 的 target 體素格重建更貴，
// 所以只在車輛離上次裁切中心超過 recrop_distance 時才重切。
class MapManager {
public:
    struct Config {
        double half_size = 50.0;         // m，xy 半邊長
        double half_height = 15.0;       // m
        double recrop_distance = 10.0;   // m
    };

    using Cloud = pcl::PointCloud<pcl::PointXYZ>;

    MapManager(Cloud::Ptr map, const Config& config) : map_(std::move(map)), config_(config) {}

    // 回傳 true 表示子地圖已更新，呼叫端需要重設 matcher 的 target
    bool Update(const Eigen::Vector3f& center);

    Cloud::ConstPtr submap() const { return submap_; }

private:
    Cloud::Ptr map_;
    Config config_;
    Cloud::Ptr submap_;
    Eigen::Vector3f crop_center_ = Eigen::Vector3f::Zero();
};

}  // namespace localization

#endif  // LOCALIZATION_MAP_MANAGER_H_
