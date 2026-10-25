#ifndef LOCALIZATION_NDT_MATCHER_H_
#define LOCALIZATION_NDT_MATCHER_H_

#include <vector>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

namespace localization {

class NdtMatcher {
public:
    struct Config {
        double resolution = 2.0;
        double step_size = 0.2;
        double transformation_epsilon = 0.01;
        int max_iterations = 35;
    };

    using Cloud = pcl::PointCloud<pcl::PointXYZ>;

    explicit NdtMatcher(const Config& config);

    void SetTarget(Cloud::ConstPtr target) { ndt_.setInputTarget(target); }

    // T_map_body = Align(source, guess)；fitness 為對應點平均平方距離，僅作配準品質參考；
    // probability 為 NDT 的轉換機率（同一幀不同起點之間可比較，越大越好）
    bool Align(Cloud::ConstPtr source, const Eigen::Matrix4f& guess, Eigen::Matrix4f& T_map_body, double& fitness,
               double& probability);

    // 從多個起點各跑一次，取機率最高者。NDT 在縱向（退化方向）的收斂域很窄，
    // 預測偏離超過 1–2 m 就拉不回來，多起點用來擴大收斂域。
    bool AlignMultiStart(Cloud::ConstPtr source, const std::vector<Eigen::Matrix4f>& guesses,
                         Eigen::Matrix4f& T_map_body, double& fitness);

private:
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
};

}  // namespace localization

#endif  // LOCALIZATION_NDT_MATCHER_H_
