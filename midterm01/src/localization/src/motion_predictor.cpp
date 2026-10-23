#include "localization/motion_predictor.h"
#include <cmath>
#include <algorithm>

namespace localization {

MotionPredictor::MotionPredictor(const MotionPredictorConfig& config)
    : config_(config) {}

void MotionPredictor::Reset() {
    has_history_ = false;
    last_pose_ = Eigen::Matrix4f::Identity();
    last_timestamp_ns_ = 0;
    last_velocity_ = Eigen::Vector3f::Zero();
    last_yaw_rate_ = 0.0;
}

double MotionPredictor::ComputeDeltaYaw(const Eigen::Matrix4f& pose_a, const Eigen::Matrix4f& pose_b) const {
    const double yaw_a = std::atan2(pose_a(1, 0), pose_a(0, 0));
    const double yaw_b = std::atan2(pose_b(1, 0), pose_b(0, 0));
    double diff = yaw_b - yaw_a;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

Eigen::Matrix4f MotionPredictor::PredictInitialGuess(const Eigen::Matrix4f& noisy_init_guess, std::int64_t timestamp_ns) {
    if (!config_.enable_motion_prior || !has_history_ || last_timestamp_ns_ <= 0) {
        return noisy_init_guess;
    }

    const double dt = static_cast<double>(timestamp_ns - last_timestamp_ns_) * 1e-9;
    if (dt <= 0.0 || dt > 1.0) {
        return noisy_init_guess;
    }

    // 恆速模型推算預測姿態
    Eigen::Matrix4f predicted_pose = last_pose_;
    predicted_pose.block<3, 1>(0, 3) += last_velocity_ * dt;

    const double last_yaw = std::atan2(last_pose_(1, 0), last_pose_(0, 0));
    const double pred_yaw = last_yaw + last_yaw_rate_ * dt;

    predicted_pose(0, 0) = std::cos(pred_yaw);
    predicted_pose(0, 1) = -std::sin(pred_yaw);
    predicted_pose(1, 0) = std::sin(pred_yaw);
    predicted_pose(1, 1) = std::cos(pred_yaw);

    // 檢驗 noisy_init_guess 與運動先驗的偏差距離
    const double deviation = (noisy_init_guess.block<3, 1>(0, 3) - predicted_pose.block<3, 1>(0, 3)).norm();

    // 若 noisy_init_guess 雜訊偏差過大，自動加大運動先驗權重
    double alpha = config_.motion_prior_weight;
    if (deviation > config_.outlier_threshold_dist) {
        alpha = std::min(1.0, alpha + 0.35);
    }

    Eigen::Matrix4f blended_pose = predicted_pose;
    blended_pose.block<3, 1>(0, 3) = (1.0 - alpha) * noisy_init_guess.block<3, 1>(0, 3) + alpha * predicted_pose.block<3, 1>(0, 3);
    
    const double noisy_yaw = std::atan2(noisy_init_guess(1, 0), noisy_init_guess(0, 0));
    double delta_yaw = noisy_yaw - pred_yaw;
    while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;
    const double final_yaw = pred_yaw + (1.0 - alpha) * delta_yaw;

    blended_pose(0, 0) = std::cos(final_yaw);
    blended_pose(0, 1) = -std::sin(final_yaw);
    blended_pose(1, 0) = std::sin(final_yaw);
    blended_pose(1, 1) = std::cos(final_yaw);

    return blended_pose;
}

bool MotionPredictor::ValidateAndCorrect(const Eigen::Matrix4f& candidate_pose,
                                        double fitness_score,
                                        std::int64_t timestamp_ns,
                                        Eigen::Matrix4f& final_pose) {
    final_pose = candidate_pose;

    if (!has_history_ || last_timestamp_ns_ <= 0) {
        return true;
    }

    const double dt = static_cast<double>(timestamp_ns - last_timestamp_ns_) * 1e-9;
    if (dt <= 0.0) return true;

    // 計算瞬間線速度與角速度
    const double displacement = (candidate_pose.block<3, 1>(0, 3) - last_pose_.block<3, 1>(0, 3)).norm();
    const double linear_speed = displacement / dt;
    const double angular_speed = std::abs(ComputeDeltaYaw(last_pose_, candidate_pose)) / dt;

    // 動態懲罰條件：速度異常突變或配準評分無效
    const bool is_physically_invalid = (linear_speed > config_.max_linear_velocity) || 
                                       (angular_speed > config_.max_angular_velocity) ||
                                       !std::isfinite(fitness_score);

    if (is_physically_invalid) {
        // 施加先驗懲罰：強行回退至運動學平滑外推姿態
        final_pose = last_pose_;
        final_pose.block<3, 1>(0, 3) += last_velocity_ * dt;
        const double fallback_yaw = std::atan2(last_pose_(1, 0), last_pose_(0, 0)) + last_yaw_rate_ * dt;
        final_pose(0, 0) = std::cos(fallback_yaw);
        final_pose(0, 1) = -std::sin(fallback_yaw);
        final_pose(1, 0) = std::sin(fallback_yaw);
        final_pose(1, 1) = std::cos(fallback_yaw);
        return false;
    }

    return true;
}

void MotionPredictor::UpdateState(const Eigen::Matrix4f& accepted_pose, std::int64_t timestamp_ns) {
    if (has_history_ && timestamp_ns > last_timestamp_ns_) {
        const double dt = static_cast<double>(timestamp_ns - last_timestamp_ns_) * 1e-9;
        if (dt > 0.0) {
            last_velocity_ = (accepted_pose.block<3, 1>(0, 3) - last_pose_.block<3, 1>(0, 3)) / dt;
            last_yaw_rate_ = ComputeDeltaYaw(last_pose_, accepted_pose) / dt;
        }
    }
    last_pose_ = accepted_pose;
    last_timestamp_ns_ = timestamp_ns;
    has_history_ = true;
}

}  // namespace localization