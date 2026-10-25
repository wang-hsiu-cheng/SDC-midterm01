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
    state_ << pose(0, 3), pose(1, 3), YawOf(pose), 0.0, initial_bias, 0.0, 0.0;
    cov_.setZero();
    cov_(0, 0) = cov_(1, 1) = config_.init_sigma_xy * config_.init_sigma_xy;
    cov_(2, 2) = std::pow(config_.init_sigma_yaw_deg * kDeg2Rad, 2);
    cov_(3, 3) = config_.init_sigma_speed * config_.init_sigma_speed;
    const double sb = bias_sigma > 0.0 ? bias_sigma : config_.init_sigma_bias;
    cov_(4, 4) = sb * sb;
    cov_(5, 5) = std::pow(config_.init_sigma_beta_deg * kDeg2Rad, 2);
    cov_(6, 6) = config_.init_sigma_accel_bias * config_.init_sigma_accel_bias;

    innov_mean_.setZero();
    innov_var_ << std::pow(config_.meas_sigma_lon, 2), std::pow(config_.meas_sigma_lat, 2),
        std::pow(config_.meas_sigma_yaw_deg * kDeg2Rad, 2);
    floor_scale_xy_ = floor_scale_yaw_ = config_.floor_adaptive ? 0.0 : 1.0;
}

void StateEstimator::Predict(double dt, double yaw_increment_raw, double forward_accel, bool stationary) {
    if (dt <= 0.0) return;
    const double x = state_(0), y = state_(1), psi = state_(2), v = state_(3), b = state_(4), beta = state_(5),
                 ba = state_(6);

    const double dpsi = yaw_increment_raw - b * dt;
    const double heading = psi + 0.5 * dpsi + beta;  // 用區間中點方向積分位置
    const double c = std::cos(heading), s = std::sin(heading);

    Matrix7 F = Matrix7::Identity();
    F(0, 2) = -v * dt * s;  F(0, 3) = dt * c;  F(0, 4) = 0.5 * v * dt * dt * s;  F(0, 5) = -v * dt * s;
    F(1, 2) = v * dt * c;   F(1, 3) = dt * s;  F(1, 4) = -0.5 * v * dt * dt * c; F(1, 5) = v * dt * c;
    F(2, 4) = -dt;
    F(3, 6) = -config_.accel_gain * dt;

    state_(0) = x + v * dt * c;
    state_(1) = y + v * dt * s;
    state_(2) = WrapAngle(psi + dpsi);
    state_(3) = std::clamp(v + config_.accel_gain * (forward_accel - ba) * dt, 0.0, config_.max_speed);

    Matrix7 Q = Matrix7::Zero();
    Q(0, 0) = Q(1, 1) = std::pow(config_.process_sigma_xy, 2) * dt;
    Q(2, 2) = std::pow(config_.process_sigma_yaw, 2) * dt;
    Q(3, 3) = std::pow(config_.process_sigma_speed, 2) * dt;
    Q(4, 4) = std::pow(config_.process_sigma_bias, 2) * dt;
    Q(5, 5) = std::pow(config_.process_sigma_beta, 2) * dt;
    Q(6, 6) = std::pow(config_.process_sigma_accel_bias, 2) * dt;
    const bool hold = stationary && config_.zupt_hold;
    if (hold) Q(0, 0) = Q(1, 1) = 0.0;
    cov_ = F * cov_ * F.transpose() + Q;
    if (!hold) {
        const double floor_xy = std::pow(config_.floor_sigma_xy * floor_scale_xy_, 2);
        const double floor_yaw = std::pow(config_.floor_sigma_yaw_deg * kDeg2Rad * floor_scale_yaw_, 2);
        for (const int i : {0, 1}) cov_(i, i) = std::max(cov_(i, i), floor_xy);
        cov_(2, 2) = std::max(cov_(2, 2), floor_yaw);
    }

    pose_(0, 3) = static_cast<float>(state_(0));
    pose_(1, 3) = static_cast<float>(state_(1));
    ApplyYawDelta(pose_, dpsi);
}

void StateEstimator::UpdateFloorScale(const Eigen::Vector3d& innovation, double c, double s) {
    // 車體座標的創新：縱向、橫向、yaw
    const Eigen::Vector3d r(c * innovation(0) + s * innovation(1), -s * innovation(0) + c * innovation(1), innovation(2));
    const double a = config_.floor_ema_alpha;
    innov_mean_ = (1.0 - a) * innov_mean_ + a * r;
    const Eigen::Vector3d deviation = r - innov_mean_;
    innov_var_ = (1.0 - config_.floor_noise_alpha) * innov_var_ + config_.floor_noise_alpha * deviation.cwiseAbs2();
    const double std_err = std::sqrt(a / (2.0 - a));  // 白雜訊下短期平均的標準差相對雜訊標準差的比例
    const auto significance = [&](int i) { return std::abs(innov_mean_(i)) / (std::sqrt(innov_var_(i)) * std_err + 1e-9); };
    const auto ramp = [&](double z) {
        return std::clamp((z - config_.floor_z_low) / (config_.floor_z_high - config_.floor_z_low), 0.0, 1.0);
    };
    floor_scale_xy_ = ramp(std::max(significance(0), significance(1)));
    floor_scale_yaw_ = ramp(significance(2));
}

void StateEstimator::UpdateStationary(double yaw_rate_mean) {
    const auto scalar_update = [this](int index, double z, double sigma) {
        const double innovation = z - state_(index);
        const double s = cov_(index, index) + sigma * sigma;
        const Vector7 k = cov_.col(index) / s;
        state_ += k * innovation;
        const Matrix7 I_KH = Matrix7::Identity() - k * Matrix7::Identity().row(index);
        cov_ = I_KH * cov_ * I_KH.transpose() + k * (sigma * sigma) * k.transpose();
        cov_ = 0.5 * (cov_ + cov_.transpose());
    };
    scalar_update(3, 0.0, config_.zupt_sigma_speed);
    scalar_update(4, yaw_rate_mean, config_.zupt_sigma_gyro_rate);
    state_(2) = WrapAngle(state_(2));
    state_(3) = std::clamp(state_(3), 0.0, config_.max_speed);
}

StateEstimator::UpdateInfo StateEstimator::Update(const Eigen::Matrix4f& T_meas, double r_scale,
                                                  const Eigen::Matrix3d& extra_cov,
                                                  bool stationary) {
    UpdateInfo info;

    const Eigen::Vector3d z(T_meas(0, 3), T_meas(1, 3), YawOf(T_meas));
    Eigen::Vector3d innovation(z(0) - state_(0), z(1) - state_(1), WrapAngle(z(2) - state_(2)));
    if (stationary && config_.zupt_freeze_yaw) innovation(2) = 0.0;

    // 基準量測雜訊沿行進方向分解：縱向（NDT 退化方向）給較大的不確定度
    const double heading = state_(2) + state_(5);
    const double c = std::cos(heading), s = std::sin(heading);
    if (config_.floor_adaptive && !(stationary && config_.zupt_hold)) UpdateFloorScale(innovation, c, s);
    Eigen::Matrix2d Rot;
    Rot << c, -s, s, c;
    Eigen::Matrix2d body_cov = Eigen::Matrix2d::Zero();
    body_cov(0, 0) = config_.meas_sigma_lon * config_.meas_sigma_lon;
    body_cov(1, 1) = config_.meas_sigma_lat * config_.meas_sigma_lat;
    Eigen::Matrix3d R0 = Eigen::Matrix3d::Zero();
    R0.topLeftCorner<2, 2>() = Rot * body_cov * Rot.transpose();
    R0(2, 2) = std::pow(config_.meas_sigma_yaw_deg * kDeg2Rad, 2);
    R0 *= r_scale;
    // 候選分散度反映這一幀的實際曖昧程度：候選一致 → 幾乎不加；縱向分歧 → 縱向 R 變大
    R0 += config_.spread_gain * extra_cov;

    Eigen::Matrix<double, 3, kStateSize> H = Eigen::Matrix<double, 3, kStateSize>::Zero();
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

    Eigen::Matrix<double, kStateSize, 3> K = cov_ * H.transpose() * S.inverse();
    // 只把 yaw 創新設 0 不夠：位置創新會透過 yaw 與位置的交叉協方差把 yaw 拉走（靜止時場景有變化，位置創新可達 1 m 以上），
    // 所以整列 K 歸零；Joseph 形式的協方差更新對任意增益都成立
    if (stationary && config_.zupt_freeze_yaw) K.row(2).setZero();
    const double yaw_before = state_(2);
    state_ += K * innovation;
    state_(2) = WrapAngle(state_(2));
    state_(3) = std::clamp(state_(3), 0.0, config_.max_speed);

    const Matrix7 I_KH = Matrix7::Identity() - K * H;
    cov_ = I_KH * cov_ * I_KH.transpose() + K * R * K.transpose();
    cov_ = 0.5 * (cov_ + cov_.transpose());

    // ---- 組回完整位姿 ----
    // 取 NDT 的 roll/pitch 與 z，與濾波後的 yaw/xy 結合
    Eigen::Matrix4f meas_aligned = T_meas;
    ApplyYawDelta(meas_aligned, WrapAngle(state_(2) - z(2)));  // 讓 NDT 旋轉的 yaw 變成濾波後的 yaw

    Eigen::Matrix4f predicted = pose_;
    ApplyYawDelta(predicted, WrapAngle(state_(2) - yaw_before));

    const Eigen::Quaternionf q_pred(Eigen::Matrix3f(predicted.block<3, 3>(0, 0)));
    const Eigen::Quaternionf q_meas(Eigen::Matrix3f(meas_aligned.block<3, 3>(0, 0)));
    const bool hold = stationary && config_.zupt_hold;
    const Eigen::Quaternionf q = q_pred.slerp(static_cast<float>(hold ? 0.0 : config_.tilt_gain), q_meas).normalized();

    pose_.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose_(0, 3) = static_cast<float>(state_(0));
    pose_(1, 3) = static_cast<float>(state_(1));
    pose_(2, 3) += static_cast<float>(hold ? 0.0 : config_.z_gain) * (T_meas(2, 3) - pose_(2, 3));
    return info;
}

void StateEstimator::Sigma(double& lon, double& lat, double& yaw_deg) const {
    const double heading = state_(2) + state_(5);
    const Eigen::Vector2d f(std::cos(heading), std::sin(heading));
    const Eigen::Vector2d l(-f.y(), f.x());
    const Eigen::Matrix2d P = cov_.topLeftCorner<2, 2>();
    lon = std::sqrt(std::max(0.0, f.dot(P * f)));
    lat = std::sqrt(std::max(0.0, l.dot(P * l)));
    yaw_deg = std::sqrt(std::max(0.0, cov_(2, 2))) / kDeg2Rad;
}

}  // namespace localization
