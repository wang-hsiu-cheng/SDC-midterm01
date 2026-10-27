#ifndef LOCALIZATION_GNSS_PRIOR_H_
#define LOCALIZATION_GNSS_PRIOR_H_

#include <cstdint>
#include <string>
#include <vector>

namespace localization {

struct GnssSample {
    std::int64_t timestamp_ns = 0;
    double x = 0.0, y = 0.0;   // 地圖座標，與 T_map_body 同一座標
    double yaw = 0.0;          // rad，來自四元數 2·atan2(qz, qw)
};

// bag 內的 GNSS（/localizer_node/gnss_pose，約 4 Hz）。助教許可 Track1／2 在演算法內使用 GNSS 先驗（使用者回報）；
// 預設關閉，只在 yaml／-p 啟用。一次讀完 bag，但只透過 Next 依時間單調取用：每次只回傳「時間戳 <= t_ns 且尚未用過的最新一筆」，
// 所以演算法看不到未來的樣本（causal），每一筆只用一次。
class GnssPrior {
public:
    bool LoadFromBag(const std::string& bag_path, const std::string& topic, std::string& error);

    // t_ns 必須單調不減。有新樣本且年齡 <= max_age_ms 時回傳 true。
    bool Next(std::int64_t t_ns, double max_age_ms, GnssSample& sample);

    std::size_t size() const { return samples_.size(); }

private:
    std::vector<GnssSample> samples_;
    std::size_t cursor_ = 0;   // 下一個尚未考慮的樣本索引
};

}  // namespace localization

#endif  // LOCALIZATION_GNSS_PRIOR_H_
