#include "localization/gnss_prior.h"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/storage_filter.hpp>

namespace localization {

bool GnssPrior::LoadFromBag(const std::string& bag_path, const std::string& topic, std::string& error) {
    samples_.clear();
    cursor_ = 0;
    try {
        rosbag2_cpp::Reader reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "sqlite3";
        reader.open(storage_options, rosbag2_cpp::ConverterOptions{"", ""});
        rosbag2_storage::StorageFilter filter;
        filter.topics.push_back(topic);
        reader.set_filter(filter);

        rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serialization;
        while (reader.has_next()) {
            const auto message = reader.read_next();
            geometry_msgs::msg::PoseStamped pose;
            rclcpp::SerializedMessage serialized(*message->serialized_data);
            serialization.deserialize_message(&serialized, &pose);

            GnssSample sample;
            sample.timestamp_ns = static_cast<std::int64_t>(pose.header.stamp.sec) * 1000000000LL + pose.header.stamp.nanosec;
            sample.x = pose.pose.position.x;
            sample.y = pose.pose.position.y;
            sample.yaw = 2.0 * std::atan2(pose.pose.orientation.z, pose.pose.orientation.w);
            samples_.push_back(sample);
        }
    } catch (const std::exception& e) {
        error = std::string("讀取 GNSS bag 失敗: ") + e.what();
        return false;
    }
    std::sort(samples_.begin(), samples_.end(), [](const GnssSample& a, const GnssSample& b) { return a.timestamp_ns < b.timestamp_ns; });
    if (samples_.empty()) {
        error = "bag 內沒有 " + topic + " 訊息";
        return false;
    }
    return true;
}

bool GnssPrior::Next(std::int64_t t_ns, double max_age_ms, GnssSample& sample) {
    bool found = false;
    GnssSample latest;
    while (cursor_ < samples_.size() && samples_[cursor_].timestamp_ns <= t_ns) {
        latest = samples_[cursor_];
        found = true;
        ++cursor_;
    }
    if (!found) return false;
    const double age_ms = static_cast<double>(t_ns - latest.timestamp_ns) * 1e-6;
    if (age_ms > max_age_ms) return false;
    sample = latest;
    return true;
}

}  // namespace localization
