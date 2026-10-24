// 地圖前處理：可選 ROI 裁切 → 體素降採樣 → 輸出 binary PCD 快取。
// 不用 pcl::VoxelGrid：全圖範圍下小 leaf（如 0.5 m）會讓其整數索引溢位而直接回傳原圖，
// 因此改以 64-bit 體素鍵的 hash 累加質心，任何 leaf 都能正確處理。
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

struct Options {
    std::string input_file;
    std::string output_file;
    double leaf_size = 1.0;
    int min_points_per_voxel = 1;
    std::vector<double> roi_min;  // [x, y, z]，空陣列 = 不裁切
    std::vector<double> roi_max;
};

struct VoxelAccumulator {
    double x = 0.0, y = 0.0, z = 0.0, intensity = 0.0;
    std::uint32_t count = 0;
};

// 每軸 21 bits（含號），leaf 0.1 m 時可涵蓋 ±104 km
constexpr std::int64_t kAxisBits = 21;
constexpr std::int64_t kAxisLimit = (std::int64_t{1} << (kAxisBits - 1)) - 1;

bool VoxelKey(const pcl::PointXYZI& p, double inv_leaf, std::int64_t& key) {
    const std::int64_t ix = static_cast<std::int64_t>(std::floor(p.x * inv_leaf));
    const std::int64_t iy = static_cast<std::int64_t>(std::floor(p.y * inv_leaf));
    const std::int64_t iz = static_cast<std::int64_t>(std::floor(p.z * inv_leaf));
    if (std::abs(ix) > kAxisLimit || std::abs(iy) > kAxisLimit || std::abs(iz) > kAxisLimit) return false;
    const std::int64_t mask = (std::int64_t{1} << kAxisBits) - 1;
    key = ((ix & mask) << (2 * kAxisBits)) | ((iy & mask) << kAxisBits) | (iz & mask);
    return true;
}

Cloud::Ptr CropRoi(const Cloud::ConstPtr& input, const Options& o) {
    auto out = std::make_shared<Cloud>();
    out->reserve(input->size());
    for (const auto& p : input->points) {
        if (p.x >= o.roi_min[0] && p.x <= o.roi_max[0] && p.y >= o.roi_min[1] && p.y <= o.roi_max[1] &&
            p.z >= o.roi_min[2] && p.z <= o.roi_max[2]) {
            out->push_back(p);
        }
    }
    return out;
}

Cloud::Ptr VoxelDownsample(const Cloud::ConstPtr& input, const Options& o, std::size_t& dropped_out_of_range) {
    const double inv_leaf = 1.0 / o.leaf_size;
    std::unordered_map<std::int64_t, VoxelAccumulator> voxels;
    voxels.reserve(input->size() / 4);
    dropped_out_of_range = 0;
    for (const auto& p : input->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        std::int64_t key = 0;
        if (!VoxelKey(p, inv_leaf, key)) {
            ++dropped_out_of_range;
            continue;
        }
        auto& v = voxels[key];
        v.x += p.x;
        v.y += p.y;
        v.z += p.z;
        v.intensity += p.intensity;
        ++v.count;
    }
    auto out = std::make_shared<Cloud>();
    out->reserve(voxels.size());
    for (const auto& [key, v] : voxels) {
        if (v.count < static_cast<std::uint32_t>(o.min_points_per_voxel)) continue;
        pcl::PointXYZI p;
        p.x = static_cast<float>(v.x / v.count);
        p.y = static_cast<float>(v.y / v.count);
        p.z = static_cast<float>(v.z / v.count);
        p.intensity = static_cast<float>(v.intensity / v.count);
        out->push_back(p);
    }
    return out;
}

bool ValidateOptions(const Options& o, const rclcpp::Logger& logger) {
    if (o.input_file.empty() || o.output_file.empty()) {
        RCLCPP_ERROR(logger, "input_file 與 output_file 不可為空");
        return false;
    }
    if (!std::isfinite(o.leaf_size) || o.leaf_size < 0.0) {
        RCLCPP_ERROR(logger, "voxel_leaf_size 必須為有限的非負值（0 = 不降採樣）");
        return false;
    }
    if (o.min_points_per_voxel < 1) {
        RCLCPP_ERROR(logger, "min_points_per_voxel 必須 >= 1");
        return false;
    }
    const bool has_roi = !o.roi_min.empty() || !o.roi_max.empty();
    if (has_roi && (o.roi_min.size() != 3 || o.roi_max.size() != 3)) {
        RCLCPP_ERROR(logger, "roi_min / roi_max 必須同時為 3 個元素 [x, y, z]，或同時留空");
        return false;
    }
    const auto in = std::filesystem::weakly_canonical(o.input_file);
    const auto out = std::filesystem::weakly_canonical(o.output_file);
    if (in == out) {
        RCLCPP_ERROR(logger, "輸出不可覆蓋原始地圖");
        return false;
    }
    return true;
}

bool Run(const rclcpp::Node::SharedPtr& node) {
    const auto logger = node->get_logger();
    Options o;
    o.input_file = node->get_parameter("input_file").as_string();
    o.output_file = node->get_parameter("output_file").as_string();
    o.leaf_size = node->get_parameter("voxel_leaf_size").as_double();
    o.min_points_per_voxel = static_cast<int>(node->get_parameter("min_points_per_voxel").as_int());
    o.roi_min = node->get_parameter("roi_min").as_double_array();
    o.roi_max = node->get_parameter("roi_max").as_double_array();
    if (!ValidateOptions(o, logger)) return false;

    const auto t0 = std::chrono::steady_clock::now();
    auto elapsed = [&t0]() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); };

    RCLCPP_INFO(logger, "讀取地圖: %s", o.input_file.c_str());
    Cloud::Ptr cloud = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile(o.input_file, *cloud) < 0 || cloud->empty()) {
        RCLCPP_ERROR(logger, "讀取失敗或地圖為空: %s", o.input_file.c_str());
        return false;
    }
    const std::size_t original = cloud->size();
    RCLCPP_INFO(logger, "載入 %zu 點（%.1f s）", original, elapsed());

    if (!o.roi_min.empty()) {
        cloud = CropRoi(cloud, o);
        RCLCPP_INFO(logger, "ROI 裁切後 %zu 點", cloud->size());
    }
    if (o.leaf_size > 0.0) {
        std::size_t dropped = 0;
        cloud = VoxelDownsample(cloud, o, dropped);
        if (dropped > 0) RCLCPP_WARN(logger, "%zu 點超出體素鍵範圍被丟棄（leaf 過小？）", dropped);
    }
    if (cloud->empty()) {
        RCLCPP_ERROR(logger, "處理後地圖為空");
        return false;
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;

    const auto parent = std::filesystem::path(o.output_file).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    if (pcl::io::savePCDFileBinary(o.output_file, *cloud) < 0) {
        RCLCPP_ERROR(logger, "寫出失敗: %s", o.output_file.c_str());
        return false;
    }
    RCLCPP_INFO(logger, "%zu -> %zu 點（leaf %.3f m, min_pts %d），已存至 %s（共 %.1f s）", original, cloud->size(),
                o.leaf_size, o.min_points_per_voxel, o.output_file.c_str(), elapsed());
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("map_modifier");
    node->declare_parameter<std::string>("input_file", "/root/data/Track1/map.pcd");
    node->declare_parameter<std::string>("output_file", "/root/data/Track1/map_downsampled_1m.pcd");
    node->declare_parameter<double>("voxel_leaf_size", 1.0);
    node->declare_parameter<int>("min_points_per_voxel", 1);
    node->declare_parameter<std::vector<double>>("roi_min", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("roi_max", std::vector<double>{});
    bool success = false;
    try {
        success = Run(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "地圖前處理失敗: %s", e.what());
    }
    rclcpp::shutdown();
    return success ? 0 : 1;
}
