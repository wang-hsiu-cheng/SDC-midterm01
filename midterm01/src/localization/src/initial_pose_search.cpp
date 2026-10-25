#include "localization/initial_pose_search.h"

#include <cmath>

#include <Eigen/Geometry>

namespace localization {

InitialPoseSearch::Result InitialPoseSearch::Search(NdtMatcher& matcher, NdtMatcher::Cloud::ConstPtr source,
                                                    const Eigen::Matrix4f& init_pose,
                                                    const std::vector<double>& yaw_centers_rad) const {
    Result best;
    const bool by_fitness = config_.criterion == "fitness";
    if (!config_.enabled || config_.yaw_step_deg <= 0.0) return best;

    const double init_yaw = std::atan2(init_pose(1, 0), init_pose(0, 0));
    const int half_steps = static_cast<int>(std::floor(config_.yaw_range_deg / config_.yaw_step_deg + 1e-9));
    for (const double center : yaw_centers_rad) {
        for (int k = -half_steps; k <= half_steps; ++k) {
            const double yaw = center + k * config_.yaw_step_deg * M_PI / 180.0;
            // 只改航向：在世界座標繞 z 轉 (yaw - init_yaw)，保留 init 的 roll／pitch
            const Eigen::Matrix3f dR =
                Eigen::AngleAxisf(static_cast<float>(yaw - init_yaw), Eigen::Vector3f::UnitZ()).toRotationMatrix();
            const Eigen::Vector3f forward(std::cos(static_cast<float>(yaw)), std::sin(static_cast<float>(yaw)), 0.0f);
            const Eigen::Vector3f left(-forward.y(), forward.x(), 0.0f);
            for (const double lon : config_.xy_offsets) {
                for (const double lat : config_.xy_offsets) {
                    Eigen::Matrix4f guess = init_pose;
                    guess.block<3, 3>(0, 0) = dR * init_pose.block<3, 3>(0, 0);
                    guess.block<3, 1>(0, 3) += static_cast<float>(lon) * forward + static_cast<float>(lat) * left;

                    NdtMatcher::Candidate candidate;
                    ++best.tried;
                    if (!matcher.Align(source, guess, candidate.pose, candidate.fitness, candidate.probability)) continue;
                    best.samples.push_back({yaw * 180.0 / M_PI, lon, lat,
                                            std::atan2(candidate.pose(1, 0), candidate.pose(0, 0)) * 180.0 / M_PI,
                                            candidate.pose(0, 3), candidate.pose(1, 3), candidate.probability,
                                            candidate.fitness});
                    const bool better = by_fitness ? candidate.fitness < best.fitness : candidate.probability > best.probability;
                    if (!best.found || better) {
                        best.found = true;
                        best.pose = candidate.pose;
                        best.probability = candidate.probability;
                        best.fitness = candidate.fitness;
                    }
                }
            }
        }
    }
    if (best.found) best.yaw_deg = std::atan2(best.pose(1, 0), best.pose(0, 0)) * 180.0 / M_PI;
    return best;
}

}  // namespace localization
