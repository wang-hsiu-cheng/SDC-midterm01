#include "localization/measurement_fusion.h"

#include <algorithm>
#include <cmath>

namespace localization {

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;

double YawOf(const Eigen::Matrix4f& T) {
    return std::atan2(static_cast<double>(T(1, 0)), static_cast<double>(T(0, 0)));
}

double Wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace

MeasurementFusion::Result MeasurementFusion::Merge(const std::vector<NdtMatcher::Candidate>& candidates,
                                                   const Eigen::Matrix4f& predicted) const {
    const std::size_t n = candidates.size();
    const double pred_yaw = YawOf(predicted);
    const Eigen::Vector2d forward(std::cos(pred_yaw), std::sin(pred_yaw));
    const Eigen::Vector2d lateral(-forward.y(), forward.x());

    double p_max = 0.0;
    for (const auto& c : candidates) p_max = std::max(p_max, c.probability);

    // 每個候選相對預測的偏移（縱向、橫向、yaw）
    std::vector<Eigen::Vector3d> offset(n);
    std::vector<double> weight(n, 0.0);
    double weight_sum = 0.0;
    std::size_t best = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const Eigen::Vector2d d(candidates[i].pose(0, 3) - predicted(0, 3), candidates[i].pose(1, 3) - predicted(1, 3));
        offset[i] = Eigen::Vector3d(forward.dot(d), lateral.dot(d), Wrap(YawOf(candidates[i].pose) - pred_yaw));
        const double ratio = p_max > 0.0 ? std::max(candidates[i].probability, 0.0) / p_max : 1.0;
        const double prior = std::exp(-0.5 * (std::pow(offset[i].x() / config_.prior_sigma_lon, 2) +
                                              std::pow(offset[i].y() / config_.prior_sigma_lat, 2) +
                                              std::pow(offset[i].z() / (config_.prior_sigma_yaw_deg * kDeg2Rad), 2)));
        weight[i] = std::pow(ratio, config_.probability_power) * prior;
        weight_sum += weight[i];
        if (weight[i] > weight[best]) best = i;
    }
    // 所有權重都被先驗壓到 0（候選全都離預測很遠）時退回機率最大者
    if (weight_sum <= 1e-12) {
        for (std::size_t i = 0; i < n; ++i) {
            if (candidates[i].probability > candidates[best].probability) best = i;
        }
        std::fill(weight.begin(), weight.end(), 0.0);
        weight[best] = 1.0;
        weight_sum = 1.0;
    }
    for (auto& w : weight) w /= weight_sum;

    // 位置、z 取加權平均；yaw 以最佳候選為參考的差值平均（避免繞圈）
    Eigen::Vector3d mean_xyz = Eigen::Vector3d::Zero();
    double yaw_delta = 0.0, eff_inv = 0.0;
    const double best_yaw = YawOf(candidates[best].pose);
    for (std::size_t i = 0; i < n; ++i) {
        mean_xyz += weight[i] * candidates[i].pose.block<3, 1>(0, 3).cast<double>();
        yaw_delta += weight[i] * Wrap(YawOf(candidates[i].pose) - best_yaw);
        eff_inv += weight[i] * weight[i];
    }

    Result result;
    result.pose = candidates[best].pose;  // roll/pitch 沿用最佳候選
    result.pose.block<3, 1>(0, 3) = mean_xyz.cast<float>();
    const Eigen::Matrix3f Rz = Eigen::AngleAxisf(static_cast<float>(yaw_delta), Eigen::Vector3f::UnitZ()).toRotationMatrix();
    result.pose.block<3, 3>(0, 0) =
        Eigen::Quaternionf(Eigen::Matrix3f(Rz * result.pose.block<3, 3>(0, 0))).normalized().toRotationMatrix();
    result.fitness = candidates[best].fitness;
    result.effective_candidates = eff_inv > 0.0 ? 1.0 / eff_inv : 1.0;

    // 候選相對加權平均的分散度（地圖座標 x, y, yaw）
    const double mean_yaw = best_yaw + yaw_delta;
    for (std::size_t i = 0; i < n; ++i) {
        const Eigen::Vector3d delta(candidates[i].pose(0, 3) - mean_xyz.x(), candidates[i].pose(1, 3) - mean_xyz.y(),
                                    Wrap(YawOf(candidates[i].pose) - mean_yaw));
        result.spread_cov += weight[i] * delta * delta.transpose();
    }
    return result;
}

}  // namespace localization
