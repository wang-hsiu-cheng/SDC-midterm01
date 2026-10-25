#ifndef LOCALIZATION_YAW_BANK_H_
#define LOCALIZATION_YAW_BANK_H_

#include <limits>
#include <string>
#include <vector>

#include "localization/state_estimator.h"

namespace localization {

// 初始 yaw 假設銀行。第 0 幀車子靜止、場景不變，NDT 對 yaw 幾乎沒有鑑別力（probability 在 ±60° 內近乎平坦），
// 單幀搜尋的結果是雜訊；車開始移動後才有視差。所以把 yaw 的決定延後：起步時複製 K 份濾波器，
// 各自套用不同的 yaw 偏移並一起追蹤，累積指標後只保留最佳者。每幀只用目前與過去的資料（causal）。
struct YawBank {
    struct Member {
        StateEstimator estimator;
        double offset_deg = 0.0;
        double fitness_sum = 0.0;
        double nis_sum = 0.0;
        double lateral_abs_sum = 0.0;  // NDT 量測相對預測的橫向偏差（地圖座標中沿預測左方向）
        double yaw_abs_sum = 0.0;      // NDT 量測相對預測的 yaw 偏差
        int count = 0;
    };

    bool active = false;
    bool done = false;
    int frames = 0;
    std::vector<Member> members;

    // score：fitness（越低越好）｜nis｜lateral｜yaw，皆為收斂幀上的平均值；尚無資料的假設排在最後
    std::size_t BestIndex(const std::string& score) const {
        std::size_t best = 0;
        double best_value = std::numeric_limits<double>::infinity();
        for (std::size_t k = 0; k < members.size(); ++k) {
            const Member& m = members[k];
            if (m.count == 0) continue;
            double sum = m.fitness_sum;
            if (score == "nis") sum = m.nis_sum;
            else if (score == "lateral") sum = m.lateral_abs_sum;
            else if (score == "yaw") sum = m.yaw_abs_sum;
            const double value = sum / m.count;
            if (value < best_value) {
                best_value = value;
                best = k;
            }
        }
        return best;
    }
};

}  // namespace localization

#endif  // LOCALIZATION_YAW_BANK_H_
