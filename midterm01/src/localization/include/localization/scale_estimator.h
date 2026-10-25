#ifndef LOCALIZATION_SCALE_ESTIMATOR_H_
#define LOCALIZATION_SCALE_ESTIMATOR_H_

#include <cstddef>
#include <vector>

namespace localization {

// 影像點雲沒有絕對尺度，而且尺度會在序列中改變（Track3 靜止時從約 7.0 一次跳到約 8.5、再漂到 9.0，
// 固定的 ply_scale 會讓 NDT 把位姿拉偏 1–2 m）。這裡用 NDT 對齊後的 fitness 因果地估計尺度：
// 每隔幾幀，以目前尺度為中心取一組等比候選，各對齊一次，取 fitness 最低者，再平滑更新。
// 呼叫端負責對每個候選尺度做對齊並回報 fitness；本類別只管候選、內插、信心判斷與平滑。
class ScaleEstimator {
public:
    struct Config {
        bool enabled = false;
        double search_ratio = 1.08;   // 相鄰候選的尺度比
        int candidates = 5;           // 候選個數（取奇數，中間那個是目前尺度）
        int interval = 3;             // 每隔幾幀搜尋一次
        double smoothing = 0.3;       // 對數尺度的指數平滑係數，1 = 直接採用量測
        double min_ratio = 0.75;      // 估計值相對標稱尺度的下限
        double max_ratio = 1.35;      // 上限，避免失控
        double min_gain = 0.01;       // 最佳 fitness 要比目前尺度的 fitness 低這個比例才更新
    };

    ScaleEstimator(const Config& config, double nominal_scale);

    double scale() const { return scale_; }
    bool enabled() const { return config_.enabled; }

    // frame_index 這一幀是否需要做尺度搜尋（第 0 幀不做：位姿還在由初始搜尋決定）
    bool Due(std::size_t frame_index) const;

    // 本次搜尋要嘗試的尺度，由小到大，中間是目前尺度
    std::vector<double> Candidates() const;

    // fitness[i] 對應 Candidates()[i]；<0 表示該候選無效（點數不足或沒收斂）。回傳更新後的尺度
    double Update(const std::vector<double>& fitness);

private:
    Config config_;
    double nominal_;
    double scale_;
};

}  // namespace localization

#endif  // LOCALIZATION_SCALE_ESTIMATOR_H_
