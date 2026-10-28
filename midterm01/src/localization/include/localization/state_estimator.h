#ifndef LOCALIZATION_STATE_ESTIMATOR_H_
#define LOCALIZATION_STATE_ESTIMATOR_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace localization {

// 平面 EKF：狀態 [x, y, yaw, v, b_g, beta, b_a]
//   v    ：沿行進方向的速度
//   b_g  ：繞上方向的陀螺儀零偏
//   beta ：行進方向相對車體 x 軸的偏角（相機安裝偏角，在線估計）
//   b_a  ：前向加速度零偏（含 IMU 姿態傾角誤差造成的重力洩漏），由 NDT 的低頻資訊估計
// z / roll / pitch 不進濾波器：NDT 對它們的約束夠強，以固定比例混合即可。
// 量測離群時只放大量測協方差（不丟棄、不截斷修正量），所以不會鎖死在錯誤的預測上。
class StateEstimator {
public:
    static constexpr int kStateSize = 7;
    using Vector7 = Eigen::Matrix<double, kStateSize, 1>;
    using Matrix7 = Eigen::Matrix<double, kStateSize, kStateSize>;

    struct Config {
        // 初始不確定度（1-sigma）
        double init_sigma_xy = 1.0;          // m
        double init_sigma_yaw_deg = 3.0;
        double init_sigma_speed = 3.0;       // m/s
        double init_sigma_bias = 0.002;      // rad/s
        double init_sigma_beta_deg = 3.0;
        double init_sigma_accel_bias = 0.05; // m/s^2

        // 連續時間過程雜訊（1-sigma / sqrt(s)）
        double process_sigma_xy = 0.05;      // m
        double process_sigma_yaw = 0.005;    // rad，陀螺儀角度隨機游走
        double process_sigma_speed = 0.1;    // m/s，未建模的加減速
        double process_sigma_bias = 1e-4;    // rad/s
        double process_sigma_beta = 1e-3;    // rad
        double process_sigma_accel_bias = 0.01;  // m/s^2

        double accel_gain = 1.0;             // 前向加速度進預測的權重；0 = 不使用加速度計
        double max_speed = 40.0;             // m/s，僅防止數值失控

        // NDT 量測雜訊基準（1-sigma，沿行進方向分解；縱向是 NDT 的退化方向）
        double meas_sigma_lon = 1.1;         // m
        double meas_sigma_lat = 0.5;         // m
        double meas_sigma_yaw_deg = 2.0;
        double spread_gain = 1.0;            // 候選分散度協方差加進 R 的倍率
        double gate_chi2 = 11.34;            // 3 自由度 99%，超過則按比例放大 R

        // 靜止時的零速／零偏觀測（ZUPT）噪聲
        double zupt_sigma_speed = 0.02;      // m/s
        double zupt_sigma_gyro_rate = 0.002; // rad/s，窗口內陀螺儀均值當零偏觀測
        double zupt_sigma_accel_bias = 0.0;  // m/s^2，窗口內前向加速度均值當加速度零偏觀測（0 = 不觀測）
        bool zupt_freeze_yaw = true;         // 靜止時 NDT 的 yaw 量測不進濾波器（停車時 NDT yaw 會殘留緩慢旋轉）
        // 靜止時鎖定：位置過程雜訊與 P 下限都不作用（濾波器對靜止期間的 NDT 量測做遞減增益的平均，
        // 不追著每幀雜訊跑；floor 在行駛段有用，但靜止時只會把雜訊灌進位姿），z 與傾角也不再混入 NDT
        bool zupt_hold = true;

        // z / roll / pitch 每幀向 NDT 結果靠攏的比例
        double z_gain = 0.5;
        double tilt_gain = 0.5;
    };

    struct UpdateInfo {
        double mahalanobis_sq = 0.0;
        double inflation = 1.0;              // 馬氏閘門與配準品質合計的 R 放大倍率
    };

    explicit StateEstimator(const Config& config) : config_(config) {}

    // initial_bias：陀螺儀零偏先驗；bias_sigma <= 0 時使用 config 的預設
    void Initialize(const Eigen::Matrix4f& pose, double initial_bias, double bias_sigma);

    // stationary：ZUPT 判定靜止；config.zupt_hold 開啟時位置不加過程雜訊、不套 P 下限
    void Predict(double dt, double yaw_increment_raw, double forward_accel, bool stationary = false);

    // 把濾波器的 yaw 與位姿旋轉 delta（rad），位置不動。yaw 假設銀行用它建立不同航向的假設
    void RotateYaw(double delta);

    // 靜止（IMU 窗口判定）時的偽量測：速度 = 0、陀螺儀均值 = 零偏、前向加速度均值 = 加速度零偏。不改位姿
    void UpdateStationary(double yaw_rate_mean, double forward_accel_mean);

    // 把位姿 z 向 target_z 拉近 gain 的比例（z 不在 EKF 狀態裡，只影響輸出與下一幀的 NDT 初值）
    void PullHeight(double target_z, double gain);

    // 外部位置量測（例如 GNSS）：地圖座標下的 (x, y)，sigma 為兩軸共用的標準差（m）
    void UpdatePosition(double x, double y, double sigma);

    // 外部航向量測（例如地圖軌跡線的方向）：delta = 量測 yaw − 目前 yaw（已包到 ±π），sigma 為量測標準差（rad）
    // yaw_only：只更新 yaw 狀態（其餘列的增益歸零），避免重複量測經交叉協方差把位置、速度、零偏帶走
    void UpdateHeading(double delta, double sigma, bool yaw_only = false);

    // T_meas：NDT 的 T_map_body；
    // extra_cov：量測在 (x, y, yaw) 地圖座標下的額外協方差（例如多起點候選的分散度）
    // stationary：ZUPT 判定靜止；config.zupt_freeze_yaw 開啟時忽略這一幀量測的 yaw
    UpdateInfo Update(const Eigen::Matrix4f& T_meas, const Eigen::Matrix3d& extra_cov, bool stationary = false);

    // 目前位置不確定度沿行進方向／橫向（m）與 yaw（deg）的 1-sigma，診斷濾波器一致性用
    void Sigma(double& lon, double& lat, double& yaw_deg) const;

    const Eigen::Matrix4f& pose() const { return pose_; }
    double speed() const { return state_(3); }
    double bias() const { return state_(4); }
    double beta() const { return state_(5); }
    double accel_bias() const { return state_(6); }
    double yaw() const { return state_(2); }

    static double WrapAngle(double a);

private:
    Config config_;
    Vector7 state_ = Vector7::Zero();
    Matrix7 cov_ = Matrix7::Identity();
    Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
};

}  // namespace localization

#endif  // LOCALIZATION_STATE_ESTIMATOR_H_
