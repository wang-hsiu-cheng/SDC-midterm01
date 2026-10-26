#ifndef LOCALIZATION_TRAIL_HEADING_H_
#define LOCALIZATION_TRAIL_HEADING_H_

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace localization {

// 地圖裡有一層貼著建圖車行駛軌跡、位於相機高度（約位姿 z ±0.25 m）的細長點雲線。
// 線的方向是道路方向，與影像點雲的尺度無關，所以可以當航向量測：NDT 的 yaw 會被單側結構拉偏，而軌跡線方向不會。
// 取位姿前後 lon_half、左右 lat_half 內、高度在位姿 z ± z_band 的圖層點，做 2D PCA；
// 必須夠細（短軸／長軸 < max_thinness）、夠長（長軸 > min_length）、夠多點，且方向與目前 yaw 的差在 gate_deg 內（線方向 mod 180°）。
// 彎道、路口的線不直，擬合不會通過 thinness 與 length 門檻，自然被排除。
class TrailHeading {
public:
    struct Config {
        bool enabled = false;
        double z_band = 0.5;           // m，相對位姿 z 的半寬
        double lon_half = 12.0;        // m，沿車頭方向的半長
        double lat_half = 3.0;         // m，橫向半寬
        int min_points = 40;
        double max_thinness = 0.25;    // 短軸標準差／長軸標準差
        double min_length = 6.0;       // m，長軸標準差×2√3 近似線段長度
        double gate_deg = 12.0;        // 線方向與目前 yaw 差的上限
        double sigma_deg = 2.0;        // 航向量測標準差
        int period_frames = 5;         // 每幾幀量一次
        bool when_stationary = false;  // 靜止時是否量測（靜止時 NDT 的 yaw 被凍結，線也會因停等累積點而變粗）
    };

    struct Result {
        bool found = false;
        double delta_rad = 0.0;        // 線方向相對目前 yaw 的差（量測 − 目前）
        int points = 0;
        double thinness = 0.0;
        double length = 0.0;
    };

    explicit TrailHeading(const Config& config) : config_(config) {}

    Result Estimate(const pcl::PointCloud<pcl::PointXYZ>& map, const Eigen::Matrix4f& pose, double yaw) const;

private:
    Config config_;
};

}  // namespace localization

#endif  // LOCALIZATION_TRAIL_HEADING_H_
