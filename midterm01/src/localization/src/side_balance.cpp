#include "localization/side_balance.h"

#include <cmath>

namespace localization {

SideBalance::Result SideBalance::Estimate(const pcl::PointCloud<pcl::PointXYZ>& map, const Eigen::Matrix4f& pose, double yaw) const {
    Result result;
    const double cx = pose(0, 3), cy = pose(1, 3), cz = pose(2, 3);
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double reach = std::max(config_.lon_max, config_.lat_max);
    for (const auto& p : map.points) {
        if (p.z < cz + config_.z_low || p.z > cz + config_.z_high) continue;
        const double dx = p.x - cx, dy = p.y - cy;
        if (std::abs(dx) > reach || std::abs(dy) > reach) continue;
        const double lon = dx * c + dy * s, lat = -dx * s + dy * c;
        if (lon <= 0.0 || lon >= config_.lon_max) continue;
        if (lat > config_.lat_min && lat < config_.lat_max) ++result.left;
        else if (lat < -config_.lat_min && lat > -config_.lat_max) ++result.right;
    }
    const int total = result.left + result.right;
    if (total < config_.min_points) return result;
    result.share = static_cast<double>(result.left) / static_cast<double>(total);
    result.active = result.share > config_.left_share_min;
    return result;
}

}  // namespace localization
