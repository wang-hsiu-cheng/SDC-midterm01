#ifndef LOCALIZATION_STATE_ESTIMATOR_H_
#define LOCALIZATION_STATE_ESTIMATOR_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace localization {

// 平面 EKF：狀態 [x, y, yaw, v, b_g, beta]
//   v    ：沿行進方向的速度
//   b_g  ：繞上方向的陀螺儀零偏
//   beta ：行進方向相對車體 x 軸的偏角（相機安裝偏角，在線估計）
// z / roll / pitch 不進濾波器：NDT 對它們的約束夠強，以固定比例混合即可。
// 量測離群時只放大量測協方差（不丟棄、不截斷修正量），所以不會鎖死在錯誤的預測上。
class StateEstimator {
public:
    struct Config {
        // 初始不確定度（1-sigma）
        double init_sigma_xy = 1.0;          // m
        double init_sigma_yaw_deg = 3.0;
        double init_sigma_speed = 3.0;       // m/s
        double init_sigma_bias = 0.002;      // rad/s
        double init_sigma_beta_deg = 3.0;

        // 連續時間過程雜訊（1-sigma / sqrt(s)）
        double process_sigma_xy = 0.05;      // m
        double process_sigma_yaw = 0.005;    // rad，陀螺儀角度隨機游走
        double process_sigma_speed = 1.5;    // m/s，未建模的加減速
        double process_sigma_bias = 1e-4;    // rad/s
        double process_sigma_beta = 1e-3;    // rad

        double accel_gain = 0.0;             // 前向加速度進預測的權重；0 = 不使用加速度計
        double max_speed = 40.0;             // m/s，僅防止數值失控

        // NDT 量測雜訊（1-sigma，沿行進方向分解；縱向是 NDT 的退化方向）
        double meas_sigma_lon = 1.0;         // m
        double meas_sigma_lat = 0.3;         // m
        double meas_sigma_yaw_deg = 1.5;
        double gate_chi2 = 11.34;            // 3 自由度 99%，超過則按比例放大 R

        // z / roll / pitch 每幀向 NDT 結果靠攏的比例
        double z_gain = 0.5;
        double tilt_gain = 0.5;
    };

    struct UpdateInfo {
        double mahalanobis_sq = 0.0;
        double inflation = 1.0;              // 最終使用的 R 放大倍率
    };

    explicit StateEstimator(const Config& config) : config_(config) {}

    // initial_bias：先驗零偏；bias_sigma <= 0 時使用 config 的預設
    void Initialize(const Eigen::Matrix4f& pose, double initial_bias, double bias_sigma);

    void Predict(double dt, double yaw_increment_raw, double forward_accel);

    // T_meas：NDT 的 T_map_body；r_scale：依配準品質額外放大 R
    UpdateInfo Update(const Eigen::Matrix4f& T_meas, double r_scale);

    const Eigen::Matrix4f& pose() const { return pose_; }
    double speed() const { return state_(3); }
    double bias() const { return state_(4); }
    double beta() const { return state_(5); }
    double yaw() const { return state_(2); }

    static double WrapAngle(double a);

private:
    Config config_;
    Eigen::Matrix<double, 6, 1> state_ = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> cov_ = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
};

}  // namespace localization

#endif  // LOCALIZATION_STATE_ESTIMATOR_H_
