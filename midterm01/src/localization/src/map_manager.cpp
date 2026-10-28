#include "localization/map_manager.h"

#include <pcl/filters/crop_box.h>

namespace localization {

// 車輛離上次裁切中心不超過 recrop_distance 就沿用子地圖（回傳 false）；否則以 center 為中心重新裁切（回傳 true，呼叫端要重設 NDT target）
bool MapManager::Update(const Eigen::Vector3f& center) {
    if (submap_ && (center - crop_center_).norm() <= config_.recrop_distance) return false;

    auto submap = std::make_shared<Cloud>();
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(map_);
    const float h = static_cast<float>(config_.half_size);
    const float hz = static_cast<float>(config_.half_height);
    crop.setMin(Eigen::Vector4f(center.x() - h, center.y() - h, center.z() - hz, 1.0f));
    crop.setMax(Eigen::Vector4f(center.x() + h, center.y() + h, center.z() + hz, 1.0f));
    crop.filter(*submap);

    submap_ = submap;
    crop_center_ = center;
    return true;
}

}  // namespace localization
