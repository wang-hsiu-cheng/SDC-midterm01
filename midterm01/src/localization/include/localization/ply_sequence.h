#ifndef LOCALIZATION_PLY_SEQUENCE_H_
#define LOCALIZATION_PLY_SEQUENCE_H_

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

// PLY 序列與位姿檔的共用讀取工具（header-only，供離線工具使用）。
namespace localization {

struct PlyFrame {
    std::int64_t timestamp_ns = 0;
    std::string path;
};

// 檔名格式 NNNNNN_ts_<ns>.ply
inline bool ParsePlyTimestamp(const std::filesystem::path& file, std::int64_t& timestamp_ns) {
    const std::string stem = file.stem().string();
    const auto pos = stem.rfind("_ts_");
    if (pos == std::string::npos) return false;
    const std::string digits = stem.substr(pos + 4);
    if (digits.empty() || !std::all_of(digits.begin(), digits.end(), [](unsigned char c) { return std::isdigit(c); })) {
        return false;
    }
    timestamp_ns = std::stoll(digits);
    return timestamp_ns > 0;
}

// 依時間戳排序；result.txt 第 i 行對應回傳的第 i 幀
inline std::vector<PlyFrame> IndexPlyDirectory(const std::string& directory) {
    std::vector<PlyFrame> frames;
    if (!std::filesystem::is_directory(directory)) return frames;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
        std::int64_t ts = 0;
        if (entry.is_regular_file() && ext == ".ply" && ParsePlyTimestamp(entry.path(), ts)) {
            frames.push_back({ts, entry.path().string()});
        }
    }
    std::sort(frames.begin(), frames.end(), [](const PlyFrame& a, const PlyFrame& b) { return a.timestamp_ns < b.timestamp_ns; });
    return frames;
}

// 每行 12 個數字（3x4 row-major）；格式錯誤的行回傳 false 並停止
inline bool LoadPoseFile(const std::string& path, std::vector<Eigen::Matrix4f>& poses, std::string& error) {
    std::ifstream in(path);
    if (!in.is_open()) {
        error = "無法開啟位姿檔: " + path;
        return false;
    }
    std::string line;
    std::size_t line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        for (int i = 0; i < 12; ++i) {
            if (!(ss >> T(i / 4, i % 4))) {
                error = "位姿檔第 " + std::to_string(line_no) + " 行少於 12 個數值";
                return false;
            }
        }
        if (!T.allFinite()) {
            error = "位姿檔第 " + std::to_string(line_no) + " 行含非有限值";
            return false;
        }
        poses.push_back(T);
    }
    return true;
}

}  // namespace localization

#endif  // LOCALIZATION_PLY_SEQUENCE_H_
