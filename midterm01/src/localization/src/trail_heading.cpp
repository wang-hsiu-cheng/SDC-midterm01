#include "localization/trail_heading.h"

#include <cmath>
#include <vector>

namespace localization {

TrailHeading::Result TrailHeading::Estimate(const pcl::PointCloud<pcl::PointXYZ>& map, const Eigen::Matrix4f& pose, double yaw) const {
    Result result;
    const double cx = pose(0, 3), cy = pose(1, 3), cz = pose(2, 3);
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double reach = std::max(config_.lon_half, config_.lat_half);

    std::vector<Eigen::Vector2d> points;
    for (const auto& p : map.points) {
        if (std::abs(p.z - cz) > config_.z_band) continue;
        const double dx = p.x - cx, dy = p.y - cy;
        if (std::abs(dx) > reach || std::abs(dy) > reach) continue;
        const double lon = dx * c + dy * s, lat = -dx * s + dy * c;
        if (std::abs(lon) > config_.lon_half || std::abs(lat) > config_.lat_half) continue;
        points.emplace_back(lon, lat);
    }
    result.points = static_cast<int>(points.size());
    if (result.points < config_.min_points) return result;

    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto& p : points) mean += p;
    mean /= static_cast<double>(points.size());
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto& p : points) cov += (p - mean) * (p - mean).transpose();
    cov /= static_cast<double>(points.size());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    const double major = std::sqrt(std::max(solver.eigenvalues()(1), 1e-12));
    const double minor = std::sqrt(std::max(solver.eigenvalues()(0), 0.0));
    result.thinness = minor / major;
    result.length = major * 2.0 * std::sqrt(3.0);
    if (result.thinness >= config_.max_thinness || result.length < config_.min_length) return result;

    const Eigen::Vector2d axis = solver.eigenvectors().col(1);
    double delta = std::atan2(axis.y(), axis.x());            // 線方向相對車頭，mod 180°
    delta = std::atan2(std::sin(2.0 * delta), std::cos(2.0 * delta)) * 0.5;
    if (std::abs(delta) * 180.0 / M_PI > config_.gate_deg) return result;
    result.delta_rad = delta;
    result.found = true;
    return result;
}

}  // namespace localization
