#include "localization/ndt_matcher.h"

namespace localization {

// 設定 NDT 參數：體素解析度、牛頓步長、收斂門檻、最大迭代次數
NdtMatcher::NdtMatcher(const Config& config) {
    ndt_.setResolution(static_cast<float>(config.resolution));
    ndt_.setStepSize(config.step_size);
    ndt_.setTransformationEpsilon(config.transformation_epsilon);
    ndt_.setMaximumIterations(config.max_iterations);
}

// 換目標子地圖：同步重建 NDT 的體素，以及 FitnessAt 用的最近點 KD-tree
void NdtMatcher::SetTarget(Cloud::ConstPtr target) {
    ndt_.setInputTarget(target);
    target_ = target;
    if (target_ && !target_->empty()) target_tree_.setInputCloud(target_);
}

// 輸出位姿與地圖的對齊度（不優化、不依賴 GNSS）：source 以 pose 變換後，到 target 最近點的平均平方距離，只計 max_range 內的點
double NdtMatcher::FitnessAt(Cloud::ConstPtr source, const Eigen::Matrix4f& pose, double max_range) const {
    if (!target_ || target_->empty() || !source || source->empty()) return -1.0;
    const double max_sq = max_range * max_range;
    double sum = 0.0;
    int count = 0;
    std::vector<int> index(1);
    std::vector<float> sq_dist(1);
    for (const auto& p : source->points) {
        const Eigen::Vector4f q = pose * Eigen::Vector4f(p.x, p.y, p.z, 1.0f);
        if (target_tree_.nearestKSearch(pcl::PointXYZ(q.x(), q.y(), q.z()), 1, index, sq_dist) > 0 && sq_dist[0] <= max_sq) {
            sum += sq_dist[0];
            ++count;
        }
    }
    return count > 0 ? sum / count : -1.0;
}

bool NdtMatcher::Align(Cloud::ConstPtr source, const Eigen::Matrix4f& guess, Eigen::Matrix4f& T_map_body,
                       double& fitness, double& probability) {
    Cloud aligned;
    ndt_.setInputSource(source);
    ndt_.align(aligned, guess);
    T_map_body = ndt_.getFinalTransformation();
    fitness = ndt_.getFitnessScore(1.0);
    probability = ndt_.getTransformationProbability();
    return ndt_.hasConverged() && T_map_body.allFinite();
}

std::vector<NdtMatcher::Candidate> NdtMatcher::AlignAll(Cloud::ConstPtr source,
                                                       const std::vector<Eigen::Matrix4f>& guesses) {
    std::vector<Candidate> candidates;
    for (const auto& guess : guesses) {
        Candidate candidate;
        if (Align(source, guess, candidate.pose, candidate.fitness, candidate.probability)) {
            candidates.push_back(candidate);
        }
    }
    return candidates;
}

}  // namespace localization
