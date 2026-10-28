#ifndef LOCALIZATION_IMU_PROCESSOR_H_
#define LOCALIZATION_IMU_PROCESSOR_H_

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace localization {

struct ImuSample {
    std::int64_t timestamp_ns = 0;
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();   // rad/s
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();  // m/s^2，比力（靜止時 +g 向上）
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // IMU 內建姿態：body -> 水平重力對齊世界
};

// 兩個時間點之間（半開區間 (t_prev, t]）的 IMU 積分結果
struct ImuInterval {
    double dt = 0.0;               // s，實際涵蓋的樣本時間
    double yaw_increment = 0.0;    // rad，繞「上方向」的角速度積分（尚未扣 bias）
    double forward_accel = 0.0;    // m/s^2，扣除重力後沿 IMU x 軸的線加速度平均
    int sample_count = 0;
};

// 一次讀完 bag 的 IMU，但只透過 Advance 依時間單調取用，確保演算法只看得到 t 以前的樣本（causal）。
class ImuProcessor {
public:
    bool LoadFromBag(const std::string& bag_path, const std::string& topic, std::string& error);

    // 推進到 t_ns，回傳上次呼叫以後的區間積分；t_ns 必須單調不減
    ImuInterval Advance(std::int64_t t_ns);

    // 在 t_ns 以前（含）的樣本中，估計靜止零偏；樣本不足 min_samples 回傳 false。不推進內部游標。
    bool EstimateYawBias(std::int64_t t_ns, int min_samples, double& bias, double& stddev) const;

    // 已推進到的最後 window_samples 個樣本是否靜止（加速度三軸標準差總和與陀螺儀三軸標準差總和皆低於門檻）。
    // 只用 Advance 已取用的樣本（causal）；靜止時 yaw_rate_mean 為窗口內繞上方向角速度均值（即零偏觀測），
    // forward_accel_mean 為窗口內扣重力後的前向加速度均值（真值為 0，剩下的就是加速度零偏）。
    bool IsStationary(int window_samples, double accel_std_max, double gyro_std_max, double& yaw_rate_mean,
                      double& forward_accel_mean) const;

    // 已推進到的最後一個樣本的姿態航向（body x 軸在 IMU 世界座標的 yaw，rad）；尚未取用任何樣本回傳 false。
    // 只用 Advance 已取用的樣本（causal）。
    bool LatestHeading(double& yaw) const;

    std::size_t size() const { return samples_.size(); }

private:
    std::vector<ImuSample> samples_;
    std::size_t cursor_ = 0;
    bool started_ = false;
    std::int64_t last_sample_ns_ = 0;
};

}  // namespace localization

#endif  // LOCALIZATION_IMU_PROCESSOR_H_
