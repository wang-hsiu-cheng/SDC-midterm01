#ifndef LOCALIZATION_MOTION_PREDICTOR_H_
#define LOCALIZATION_MOTION_PREDICTOR_H_

#include <cstdint>
#include <deque>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace localization {

struct MotionPredictorConfig {
    bool enable_motion_prior = true;
    double max_linear_velocity = 35.0;     // 公尺/秒 (超過視為配準發散)
    double max_angular_velocity = 1.5;     // 弧度/秒 (超過視為配準發散)
    double motion_prior_weight = 0.5;      // init_guess 與運動先驗融合權重 (0: 全用 init_guess, 1: 全用先驗)
    double outlier_threshold_dist = 3.0;   // 與預測姿態距離超過此值觸發懲罰修正 (m)
};

class MotionPredictor {
public:
    explicit MotionPredictor(const MotionPredictorConfig& config = MotionPredictorConfig());

    // 重設狀態
    void Reset();

    // 結合前幾幀姿態與當前 init_guess，生成平滑抗雜訊的配準初值
    Eigen::Matrix4f PredictInitialGuess(const Eigen::Matrix4f& noisy_init_guess, std::int64_t timestamp_ns);

    // 對 G-ICP 配準後的 candidate 進行物理動力學檢驗與動態懲罰修復
    bool ValidateAndCorrect(const Eigen::Matrix4f& candidate_pose,
                            double fitness_score,
                            std::int64_t timestamp_ns,
                            Eigen::Matrix4f& final_pose);

    // 成功接受姿態後，更新內部運動學歷史
    void UpdateState(const Eigen::Matrix4f& accepted_pose, std::int64_t timestamp_ns);

private:
    double ComputeDeltaYaw(const Eigen::Matrix4f& pose_a, const Eigen::Matrix4f& pose_b) const;

    MotionPredictorConfig config_;
    bool has_history_ = false;
    
    Eigen::Matrix4f last_pose_ = Eigen::Matrix4f::Identity();
    std::int64_t last_timestamp_ns_ = 0;

    Eigen::Vector3f last_velocity_ = Eigen::Vector3f::Zero();
    double last_yaw_rate_ = 0.0;
};

}  // namespace localization

#endif  // LOCALIZATION_MOTION_PREDICTOR_H_