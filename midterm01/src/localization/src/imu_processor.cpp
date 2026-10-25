#include "localization/imu_processor.h"

#include <algorithm>
#include <cmath>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace localization {

bool ImuProcessor::LoadFromBag(const std::string& bag_path, const std::string& topic, std::string& error) {
    samples_.clear();
    cursor_ = 0;
    started_ = false;
    try {
        rosbag2_cpp::Reader reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "sqlite3";
        reader.open(storage_options, rosbag2_cpp::ConverterOptions{"", ""});
        rosbag2_storage::StorageFilter filter;
        filter.topics.push_back(topic);
        reader.set_filter(filter);

        rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
        while (reader.has_next()) {
            const auto message = reader.read_next();
            sensor_msgs::msg::Imu imu;
            rclcpp::SerializedMessage serialized(*message->serialized_data);
            serialization.deserialize_message(&serialized, &imu);

            ImuSample sample;
            // 用 header.stamp 與 PLY 檔名時間戳同一時鐘
            sample.timestamp_ns = static_cast<std::int64_t>(imu.header.stamp.sec) * 1000000000LL + imu.header.stamp.nanosec;
            sample.gyro = {imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z};
            sample.accel = {imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z};
            sample.orientation = Eigen::Quaterniond(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
            if (sample.orientation.norm() < 0.5) sample.orientation = Eigen::Quaterniond::Identity();
            sample.orientation.normalize();
            samples_.push_back(sample);
        }
    } catch (const std::exception& e) {
        error = std::string("讀取 IMU bag 失敗: ") + e.what();
        return false;
    }
    std::sort(samples_.begin(), samples_.end(),
              [](const ImuSample& a, const ImuSample& b) { return a.timestamp_ns < b.timestamp_ns; });
    if (samples_.empty()) {
        error = "bag 內沒有 " + topic + " 訊息";
        return false;
    }
    return true;
}

ImuInterval ImuProcessor::Advance(std::int64_t t_ns) {
    constexpr double kGravity = 9.81;
    ImuInterval out;
    double forward_sum = 0.0;
    while (cursor_ < samples_.size() && samples_[cursor_].timestamp_ns <= t_ns) {
        const ImuSample& s = samples_[cursor_];
        if (!started_) {
            last_sample_ns_ = s.timestamp_ns;
            started_ = true;
        }
        const double dt = static_cast<double>(s.timestamp_ns - last_sample_ns_) * 1e-9;
        last_sample_ns_ = s.timestamp_ns;
        if (dt > 0.0 && dt < 0.5) {
            // 世界上方向在 IMU 座標系的表示；用 IMU 內建姿態而不是加速度低通，
            // 加減速時後者會把縱向加速度誤當成傾角
            const Eigen::Vector3d up = s.orientation.conjugate() * Eigen::Vector3d::UnitZ();
            out.yaw_increment += s.gyro.dot(up) * dt;
            forward_sum += (s.accel - kGravity * up).x() * dt;
            out.dt += dt;
            ++out.sample_count;
        }
        ++cursor_;
    }
    if (out.dt > 0.0) out.forward_accel = forward_sum / out.dt;
    return out;
}

bool ImuProcessor::EstimateYawBias(std::int64_t t_ns, int min_samples, double& bias, double& stddev) const {
    double sum = 0.0, sum_sq = 0.0;
    int n = 0;
    for (const auto& s : samples_) {
        if (s.timestamp_ns > t_ns) break;
        const double w = s.gyro.dot(s.orientation.conjugate() * Eigen::Vector3d::UnitZ());
        sum += w;
        sum_sq += w * w;
        ++n;
    }
    if (n < min_samples) return false;
    bias = sum / n;
    stddev = std::sqrt(std::max(0.0, sum_sq / n - bias * bias));
    return true;
}

}  // namespace localization
