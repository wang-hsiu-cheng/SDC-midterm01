#ifndef LOCALIZATION_INITIAL_POSE_SEARCH_H_
#define LOCALIZATION_INITIAL_POSE_SEARCH_H_

#include <vector>

#include <Eigen/Dense>

#include "localization/ndt_matcher.h"

namespace localization {

// 第 0 幀的初始位姿搜尋。init_guess 帶雜訊（Track3 的航向偏約 20°、Track2 約 8°），
// 偏差超出 NDT 收斂域時，後續幀要花上千幀才能被地圖拉回來。
// 這裡對多個航向／位置假設各跑一次 NDT，由地圖匹配（transformation probability）選最佳者；
// IMU 姿態的航向只負責「提出候選中心」，不直接決定結果。
class InitialPoseSearch {
public:
    struct Config {
        bool enabled = true;
        double yaw_range_deg = 30.0;   // 每個中心左右各搜尋的範圍
        double yaw_step_deg = 5.0;
        std::vector<double> xy_offsets{0.0, 2.0, -2.0};  // m，沿候選航向的縱向與橫向各取這些偏移（做格狀組合）
    };

    struct Result {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        double yaw_deg = 0.0;
        double probability = 0.0;
        double fitness = 0.0;
        int tried = 0;
        bool found = false;
    };

    explicit InitialPoseSearch(const Config& config) : config_(config) {}

    // yaw_centers_rad：候選航向中心（如 init_guess 的航向與 IMU 姿態航向）。
    // 旋轉只改航向（繞世界 z），roll／pitch／z 沿用 init_pose。matcher 的 target 須已設好。
    Result Search(NdtMatcher& matcher, NdtMatcher::Cloud::ConstPtr source, const Eigen::Matrix4f& init_pose,
                  const std::vector<double>& yaw_centers_rad) const;

private:
    Config config_;
};

}  // namespace localization

#endif  // LOCALIZATION_INITIAL_POSE_SEARCH_H_
