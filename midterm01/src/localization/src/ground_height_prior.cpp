#include "localization/ground_height_prior.h"

#include <algorithm>
#include <vector>

namespace localization {

GroundHeightPrior::Result GroundHeightPrior::Estimate(const pcl::PointCloud<pcl::PointXYZ>& map, const Eigen::Matrix4f& pose) const {
    Result result;
    const double cx = pose(0, 3), cy = pose(1, 3), cz = pose(2, 3);
    const double r2 = config_.radius * config_.radius;
    std::vector<float> zs;
    for (const auto& p : map.points) {
        if (p.z < cz + config_.z_low || p.z > cz + config_.z_high) continue;
        const double dx = p.x - cx, dy = p.y - cy;
        if (dx * dx + dy * dy > r2) continue;
        zs.push_back(p.z);
    }
    result.points = static_cast<int>(zs.size());
    if (result.points < config_.min_points) return result;
    const std::size_t idx = zs.size() / 5;   // 20% 分位數：路面
    std::nth_element(zs.begin(), zs.begin() + idx, zs.end());
    result.ground_z = static_cast<double>(zs[idx]);
    result.target_z = result.ground_z + config_.camera_height;
    result.found = true;
    return result;
}

}  // namespace localization
