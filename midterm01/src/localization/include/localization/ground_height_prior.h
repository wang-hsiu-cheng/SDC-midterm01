#ifndef LOCALIZATION_GROUND_HEIGHT_PRIOR_H_
#define LOCALIZATION_GROUND_HEIGHT_PRIOR_H_

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace localization {

// 位姿 z 完全由 NDT 的 z 以 z_gain 慢慢混入、IMU 不約束，轉彎後會往上漂（Track3：位姿高於地圖地面從 1.7 m 漲到 2.0～2.45 m，
// 相機點雲的地面因此落在 LiDAR 地面上方 0.5～0.9 m，STATUS §4-34）。車輛離路面的高度是常數（相機高度），
// 所以用地圖路面約束 z：z_target = 位姿 xy 周圍的地圖地面 + camera_height。只用地圖，不用 GNSS。
class GroundHeightPrior {
public:
    struct Config {
        bool enabled = false;
        double camera_height = 1.85;   // m，相機（位姿）離路面的高度；Track3 行駛段 100–700 幀的位姿−地面為 1.71～1.96 m
        double radius = 4.0;           // m，估地面用的水平範圍
        double z_low = -4.0;           // m，相對目前位姿 z 的搜尋範圍（略過車頂以上的物體）
        double z_high = -0.5;
        int min_points = 20;
        double gain = 0.5;             // 每幀把位姿 z 拉向 z_target 的比例
    };

    struct Result {
        bool found = false;
        double ground_z = 0.0;
        double target_z = 0.0;
        int points = 0;
    };

    explicit GroundHeightPrior(const Config& config) : config_(config) {}

    Result Estimate(const pcl::PointCloud<pcl::PointXYZ>& map, const Eigen::Matrix4f& pose) const;

private:
    Config config_;
};

}  // namespace localization

#endif  // LOCALIZATION_GROUND_HEIGHT_PRIOR_H_
