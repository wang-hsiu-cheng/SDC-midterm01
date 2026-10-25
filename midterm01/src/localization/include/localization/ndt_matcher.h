#ifndef LOCALIZATION_NDT_MATCHER_H_
#define LOCALIZATION_NDT_MATCHER_H_

#include <vector>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
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

    void SetTarget(Cloud::ConstPtr target);

    // 把 source 以 pose 變換後，對 target 的最近點平均平方距離（只計 max_range 內的點，與 Align 的 fitness 同定義）。
    // 不做優化：用來量「最終輸出位姿」和地圖的對齊度（不依賴 GNSS 的評估指標）；沒有 target 或無對應點時回傳 -1
    double FitnessAt(Cloud::ConstPtr source, const Eigen::Matrix4f& pose, double max_range = 1.0) const;

    // T_map_body = Align(source, guess)；fitness 為對應點平均平方距離，僅作配準品質參考；
    // probability 為 NDT 的轉換機率（同一幀不同起點之間可比較，越大越好）
    bool Align(Cloud::ConstPtr source, const Eigen::Matrix4f& guess, Eigen::Matrix4f& T_map_body, double& fitness,
               double& probability);

    struct Candidate {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        double fitness = 0.0;
        double probability = 0.0;
    };

    // 從多個起點各跑一次並回傳所有收斂的結果。NDT 在縱向（退化方向）的收斂域很窄，
    // 預測偏離超過 1–2 m 就拉不回來，多起點用來擴大收斂域；如何挑選或合併由呼叫端決定
    // （只取機率最大者會在不同模式間翻轉，把 ~1 m 的跳動當成量測）。
    std::vector<Candidate> AlignAll(Cloud::ConstPtr source, const std::vector<Eigen::Matrix4f>& guesses);

private:
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    Cloud::ConstPtr target_;
    pcl::KdTreeFLANN<pcl::PointXYZ> target_tree_;
};

}  // namespace localization

#endif  // LOCALIZATION_NDT_MATCHER_H_
