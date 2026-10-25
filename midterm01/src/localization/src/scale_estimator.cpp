#include "localization/scale_estimator.h"

#include <algorithm>
#include <cmath>

namespace localization {

ScaleEstimator::ScaleEstimator(const Config& config, double nominal_scale)
    : config_(config), nominal_(nominal_scale), scale_(nominal_scale) {
    if (config_.candidates < 3) config_.candidates = 3;
    if (config_.candidates % 2 == 0) ++config_.candidates;
    if (config_.interval < 1) config_.interval = 1;
}

bool ScaleEstimator::Due(std::size_t frame_index) const {
    return config_.enabled && frame_index > 0 && frame_index % static_cast<std::size_t>(config_.interval) == 0;
}

std::vector<double> ScaleEstimator::Candidates() const {
    const int half = config_.candidates / 2;
    std::vector<double> out;
    for (int k = -half; k <= half; ++k) out.push_back(scale_ * std::pow(config_.search_ratio, k));
    return out;
}

double ScaleEstimator::Update(const std::vector<double>& fitness) {
    const int n = static_cast<int>(fitness.size());
    const int center = n / 2;
    if (n != config_.candidates || fitness[center] <= 0.0) return scale_;

    int best = -1;
    for (int i = 0; i < n; ++i) {
        if (fitness[i] > 0.0 && (best < 0 || fitness[i] < fitness[best])) best = i;
    }
    // 最佳候選沒有明顯優於目前尺度 → 這一幀的量測沒有資訊，不更新
    if (best < 0 || best == center || fitness[best] > fitness[center] * (1.0 - config_.min_gain)) return scale_;

    // 對數尺度上的等間距，用相鄰三點做拋物線內插；在邊界或相鄰無效時直接取該候選
    double offset = 0.0;
    if (best > 0 && best < n - 1 && fitness[best - 1] > 0.0 && fitness[best + 1] > 0.0) {
        const double a = fitness[best - 1], b = fitness[best], c = fitness[best + 1];
        const double denom = a - 2.0 * b + c;
        if (denom > 0.0) offset = std::clamp(0.5 * (a - c) / denom, -0.5, 0.5);
    }
    const double log_step = std::log(config_.search_ratio);
    const double log_meas = std::log(scale_) + (best - center + offset) * log_step;
    double log_scale = std::log(scale_) + config_.smoothing * (log_meas - std::log(scale_));
    log_scale = std::clamp(log_scale, std::log(nominal_ * config_.min_ratio), std::log(nominal_ * config_.max_ratio));
    scale_ = std::exp(log_scale);
    return scale_;
}

}  // namespace localization
