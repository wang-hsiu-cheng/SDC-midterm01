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

bool NdtMatcher::AlignMultiStart(Cloud::ConstPtr source, const std::vector<Eigen::Matrix4f>& guesses,
                                 Eigen::Matrix4f& T_map_body, double& fitness) {
    bool found = false;
    double best_probability = -1.0;
    for (const auto& guess : guesses) {
        Eigen::Matrix4f result;
        double candidate_fitness = 0.0, probability = 0.0;
        if (!Align(source, guess, result, candidate_fitness, probability)) continue;
        if (!found || probability > best_probability) {
            found = true;
            best_probability = probability;
            T_map_body = result;
            fitness = candidate_fitness;
        }
    }
    return found;
}

}  // namespace localization
