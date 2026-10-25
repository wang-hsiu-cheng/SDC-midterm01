#include "localization/state_estimator.h"

#include <algorithm>
#include <cmath>

namespace localization {

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;

double YawOf(const Eigen::Matrix4f& T) {
    return std::atan2(static_cast<double>(T(1, 0)), static_cast<double>(T(0, 0)));
}

// 左乘繞世界 z 軸的旋轉：只改 yaw，保留 roll/pitch 結構
void ApplyYawDelta(Eigen::Matrix4f& T, double delta) {
    const Eigen::Matrix3f Rz = Eigen::AngleAxisf(static_cast<float>(delta), Eigen::Vector3f::UnitZ()).toRotationMatrix();
    const Eigen::Matrix3f R = Rz * T.block<3, 3>(0, 0);
    T.block<3, 3>(0, 0) = Eigen::Quaternionf(R).normalized().toRotationMatrix();
}

}  // namespace

double StateEstimator::WrapAngle(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

void StateEstimator::Initialize(const Eigen::Matrix4f& pose, double initial_bias, double bias_sigma) {
    pose_ = pose;
    state_ << pose(0, 3), pose(1, 3), YawOf(pose), 0.0, initial_bias, 0.0;
    cov_.setZero();
    cov_(0, 0) = cov_(1, 1) = config_.init_sigma_xy * config_.init_sigma_xy;
    cov_(2, 2) = std::pow(config_.init_sigma_yaw_deg * kDeg2Rad, 2);
    cov_(3, 3) = config_.init_sigma_speed * config_.init_sigma_speed;
    const double sb = bias_sigma > 0.0 ? bias_sigma : config_.init_sigma_bias;
    cov_(4, 4) = sb * sb;
    cov_(5, 5) = std::pow(config_.init_sigma_beta_deg * kDeg2Rad, 2);
}

void StateEstimator::Predict(double dt, double yaw_increment_raw, double forward_accel) {
    if (dt <= 0.0) return;
    const double x = state_(0), y = state_(1), psi = state_(2), v = state_(3), b = state_(4), beta = state_(5);

    const double dpsi = yaw_increment_raw - b * dt;
    const double heading = psi + 0.5 * dpsi + beta;  // 用區間中點方向積分位置
    const double c = std::cos(heading), s = std::sin(heading);

    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F(0, 2) = -v * dt * s;  F(0, 3) = dt * c;  F(0, 4) = 0.5 * v * dt * dt * s;  F(0, 5) = -v * dt * s;
    F(1, 2) = v * dt * c;   F(1, 3) = dt * s;  F(1, 4) = -0.5 * v * dt * dt * c; F(1, 5) = v * dt * c;
    F(2, 4) = -dt;

    state_(0) = x + v * dt * c;
    state_(1) = y + v * dt * s;
    state_(2) = WrapAngle(psi + dpsi);
    state_(3) = std::clamp(v + config_.accel_gain * forward_accel * dt, 0.0, config_.max_speed);

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q(0, 0) = Q(1, 1) = std::pow(config_.process_sigma_xy, 2) * dt;
    Q(2, 2) = std::pow(config_.process_sigma_yaw, 2) * dt;
    Q(3, 3) = std::pow(config_.process_sigma_speed, 2) * dt;
    Q(4, 4) = std::pow(config_.process_sigma_bias, 2) * dt;
    Q(5, 5) = std::pow(config_.process_sigma_beta, 2) * dt;
    cov_ = F * cov_ * F.transpose() + Q;

    pose_(0, 3) = static_cast<float>(state_(0));
    pose_(1, 3) = static_cast<float>(state_(1));
    ApplyYawDelta(pose_, dpsi);
}

StateEstimator::UpdateInfo StateEstimator::Update(const Eigen::Matrix4f& T_meas, double r_scale) {
    UpdateInfo info;

    const Eigen::Vector3d z(T_meas(0, 3), T_meas(1, 3), YawOf(T_meas));
    Eigen::Vector3d innovation(z(0) - state_(0), z(1) - state_(1), WrapAngle(z(2) - state_(2)));

    // 量測雜訊沿行進方向分解：縱向（NDT 退化方向）給較大的不確定度
    const double heading = state_(2) + state_(5);
    const double c = std::cos(heading), s = std::sin(heading);
    Eigen::Matrix2d Rot;
    Rot << c, -s, s, c;
    Eigen::Matrix2d body_cov = Eigen::Matrix2d::Zero();
    body_cov(0, 0) = config_.meas_sigma_lon * config_.meas_sigma_lon;
    body_cov(1, 1) = config_.meas_sigma_lat * config_.meas_sigma_lat;
    Eigen::Matrix3d R0 = Eigen::Matrix3d::Zero();
    R0.topLeftCorner<2, 2>() = Rot * body_cov * Rot.transpose();
    R0(2, 2) = std::pow(config_.meas_sigma_yaw_deg * kDeg2Rad, 2);
    R0 *= std::max(1.0, r_scale);

    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 0) = H(1, 1) = H(2, 2) = 1.0;
    const Eigen::Matrix3d HPHt = H * cov_ * H.transpose();

    // 馬氏距離超過門檻 → 按比例放大 R（限制單次影響而不是拒絕），預測錯了也能靠 P 成長慢慢拉回
    Eigen::Matrix3d S = HPHt + R0;
    info.mahalanobis_sq = innovation.dot(S.ldlt().solve(innovation));
    double inflation = 1.0;
    if (info.mahalanobis_sq > config_.gate_chi2) {
        inflation = info.mahalanobis_sq / config_.gate_chi2;
    }
    const Eigen::Matrix3d R = R0 * inflation;
    info.inflation = inflation * std::max(1.0, r_scale);
    S = HPHt + R;

    const Eigen::Matrix<double, 6, 3> K = cov_ * H.transpose() * S.inverse();
    const double yaw_before = state_(2);
    state_ += K * innovation;
    state_(2) = WrapAngle(state_(2));
    state_(3) = std::clamp(state_(3), 0.0, config_.max_speed);

    const Eigen::Matrix<double, 6, 6> I_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
    cov_ = I_KH * cov_ * I_KH.transpose() + K * R * K.transpose();
    cov_ = 0.5 * (cov_ + cov_.transpose());

    // ---- 組回完整位姿 ----
    // 取 NDT 的 roll/pitch 與 z，與濾波後的 yaw/xy 結合
    const double yaw_meas = z(2);
    Eigen::Matrix4f meas_aligned = T_meas;
    ApplyYawDelta(meas_aligned, WrapAngle(state_(2) - yaw_meas));  // 讓 NDT 旋轉的 yaw 變成濾波後的 yaw

    Eigen::Matrix4f predicted = pose_;
    ApplyYawDelta(predicted, WrapAngle(state_(2) - yaw_before));

    const Eigen::Quaternionf q_pred(Eigen::Matrix3f(predicted.block<3, 3>(0, 0)));
    const Eigen::Quaternionf q_meas(Eigen::Matrix3f(meas_aligned.block<3, 3>(0, 0)));
    const Eigen::Quaternionf q = q_pred.slerp(static_cast<float>(config_.tilt_gain), q_meas).normalized();

    pose_.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose_(0, 3) = static_cast<float>(state_(0));
    pose_(1, 3) = static_cast<float>(state_(1));
    pose_(2, 3) += static_cast<float>(config_.z_gain) * (T_meas(2, 3) - pose_(2, 3));
    return info;
}

}  // namespace localization
