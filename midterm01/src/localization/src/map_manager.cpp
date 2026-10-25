#include "localization/map_manager.h"

#include <pcl/filters/crop_box.h>

namespace localization {

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
