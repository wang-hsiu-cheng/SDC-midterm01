#ifndef LOCALIZATION_SIDE_BALANCE_H_
#define LOCALIZATION_SIDE_BALANCE_H_

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace localization {

// NDT 的目標函數由「有結構的一側」主導：車前方一側出現大量垂直結構時，yaw 會被那一側往同方向拉（Track3 接近紅燈時
// 左側結構進入視野，NDT yaw 被往逆時針拉約 +4°，STATUS §4-19、§4-29）。這裡只用地圖算位姿前方左右兩側的垂直結構點數，
// 左側比例 L/(L+R) 超過門檻就替這一幀的 yaw 量測加額外的標準差，讓陀螺儀預測（與軌跡線）主導航向。只用地圖，不用 GNSS。
class SideBalance {
public:
    struct Config {
        bool enabled = false;
        double left_share_min = 0.45;   // 左側比例 L/(L+R) 超過此值才降權
        double lon_max = 25.0;          // m，前方範圍
        double lat_min = 2.0;           // m，略過車旁 lat_min 內（路面物件）
        double lat_max = 20.0;          // m
        double z_low = -0.5;            // m，相對位姿 z（略過地面）
        double z_high = 3.0;
        int min_points = 200;           // 左右合計太少時不判斷
        double yaw_sigma_deg = 6.0;     // 額外的 yaw 量測標準差（與基準 σ 以方差相加）
    };

    struct Result {
        int left = 0, right = 0;
        double share = 0.0;
        bool active = false;
    };

    explicit SideBalance(const Config& config) : config_(config) {}

    Result Estimate(const pcl::PointCloud<pcl::PointXYZ>& map, const Eigen::Matrix4f& pose, double yaw) const;

private:
    Config config_;
};

}  // namespace localization

#endif  // LOCALIZATION_SIDE_BALANCE_H_
