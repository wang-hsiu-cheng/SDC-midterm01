#include "localization/ndt_matcher.h"

namespace localization {

NdtMatcher::NdtMatcher(const Config& config) {
    ndt_.setResolution(static_cast<float>(config.resolution));
    ndt_.setStepSize(config.step_size);
    ndt_.setTransformationEpsilon(config.transformation_epsilon);
    ndt_.setMaximumIterations(config.max_iterations);
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
