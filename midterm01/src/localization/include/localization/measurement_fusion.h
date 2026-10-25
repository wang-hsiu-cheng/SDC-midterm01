#ifndef LOCALIZATION_MEASUREMENT_FUSION_H_
#define LOCALIZATION_MEASUREMENT_FUSION_H_

#include <vector>

#include <Eigen/Dense>

#include "localization/ndt_matcher.h"

namespace localization {

// 把多起點 NDT 的候選合成一個量測，並給出這一幀量測的經驗協方差。
// 用 probability 比值與預測先驗做軟加權，而不是取 argmax：後者在不同模式之間翻轉時，
// 量測會每幀跳 ~1 m，且濾波器完全不知道這一幀其實是曖昧的。
class MeasurementFusion {
public:
    struct Config {
        double probability_power = 4.0;    // w ∝ (p / p_max)^power；越大越接近 argmax
        // 以預測為中心的高斯先驗（沿行進方向分解），避免被系統性偏後的候選拉走
        double prior_sigma_lon = 2.0;      // m
        double prior_sigma_lat = 1.0;      // m
        double prior_sigma_yaw_deg = 5.0;
    };

    struct Result {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        double fitness = 0.0;
        Eigen::Matrix3d spread_cov = Eigen::Matrix3d::Zero();  // 地圖座標下 (x, y, yaw) 的候選分散度
        double effective_candidates = 1.0;                     // 1 / sum(w^2)
    };

    explicit MeasurementFusion(const Config& config) : config_(config) {}

    // predicted：預測位姿，作為先驗中心與行進方向參考；candidates 不可為空
    Result Merge(const std::vector<NdtMatcher::Candidate>& candidates, const Eigen::Matrix4f& predicted) const;

private:
    Config config_;
};

}  // namespace localization

#endif  // LOCALIZATION_MEASUREMENT_FUSION_H_
