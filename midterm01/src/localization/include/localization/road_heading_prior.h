#ifndef LOCALIZATION_ROAD_HEADING_PRIOR_H_
#define LOCALIZATION_ROAD_HEADING_PRIOR_H_

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace localization {

// 靜止起步時第 0 幀的 NDT 對 yaw 沒有鑑別力（Track2 的 yaw 景觀是平的），init_guess 的 yaw 又偏了十幾度，
// 而輸出在車動起來之前不能事後修正。車停在路上時航向通常和道路方向一致，道路方向可以直接從地圖的牆面結構算出：
// 取起點周圍離地面以上的點，建 2D 點密度影像，牆面是影像的邊緣，邊緣方向（梯度方向 +90°）的直方圖主峰就是牆面主方向（mod 180°），
// 180° 的模糊用 init_guess 的 yaw 解（取最近者）。只用地圖與 init_guess，不用 GNSS。
// 與 init_guess 差超過 max_deviation_deg 就放棄（走廊不明顯的起點，例如路口、空曠處）。
class RoadHeadingPrior {
public:
    struct Config {
        bool enabled = false;
        double radius = 30.0;            // m，取點範圍（正方形半邊長）
        double cell = 0.5;               // m，影像格子
        double z_min = -1.0;             // m，相對 init_pose 高度的下限（略過地面）
        double z_max = 6.0;
        double max_deviation_deg = 25.0;
    };

    struct Result {
        bool found = false;
        double wall_deg = 0.0;           // 牆面主方向（mod 180°，地圖座標）
        double yaw_rad = 0.0;            // 解完 180° 模糊後的航向
        double deviation_deg = 0.0;      // 與 init_guess 的 yaw 差
        int points = 0;
    };

    explicit RoadHeadingPrior(const Config& config) : config_(config) {}

    Result Estimate(const pcl::PointCloud<pcl::PointXYZ>& map, const Eigen::Matrix4f& init_pose) const;

private:
    Config config_;
};

}  // namespace localization

#endif  // LOCALIZATION_ROAD_HEADING_PRIOR_H_
