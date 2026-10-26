// 配準器比較基準：同一個最簡追蹤流程（等速預測 → 配準 → 寫出），只替換配準器，
// 用來判斷 NDT 與 G-ICP 何者適合作為重構後的核心。刻意不含任何限幅或濾波，
// 以免掩蓋配準器本身的差異。
//
// 用法：
//   matcher_bench --method ndt|gicp --map <pcd> --ply_dir <dir> --init <init_guess.txt>
//                 --out <result.txt> [--stats <stats.csv>] [--max_frames N] [--scale 8]
//                 [--source_voxel 0.5] [--max_range 0] [--epsilon 0.01] [--submap_half 50] [--recrop_dist 10]
//                 [--ndt_resolution 3] [--ndt_step 0.2] [--max_iter 35]
//                 [--gicp_max_corr 2.0] [--gicp_k 20] [--reject_dist 3.0] [--target_voxel 0]
//   guess 模式（不追蹤，逐行獨立配準，用來量測收斂域與偏差）：
//                 --guess_file <每行: 幀索引 + 12 個 3x4 位姿值>，此時 --init 可省略，
//                 out 每行輸出「幀索引 + 12 個位姿值」
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct Options {
    std::string method = "ndt";
    std::string map, ply_dir, init, out, stats, guess_file;
    int max_frames = 0;
    double scale = 8.0;
    double source_voxel = 0.5;
    double max_range = 0.0;     // m，縮放後水平距離上限（與管線的 input_max_range 相同）；0 = 不過濾
    double ndt_epsilon = 0.01;  // NDT 收斂門檻（管線的 ndt_transformation_epsilon）
    double submap_half = 50.0;
    double submap_half_z = 15.0;
    double recrop_dist = 10.0;
    double ndt_resolution = 3.0;
    double ndt_step = 0.2;
    int max_iter = 35;
    double gicp_max_corr = 2.0;
    int gicp_k = 20;
    double reject_dist = 3.0;
    double target_voxel = 0.0;  // >0 時對裁切後的子地圖降採樣（全圖直接 voxel 會整數溢位）
};

bool ParseArgs(int argc, char** argv, Options& o) {
    std::map<std::string, std::string> kv;
    for (int i = 1; i + 1 < argc; i += 2) kv[argv[i]] = argv[i + 1];
    auto str = [&](const char* k, std::string& v) { if (kv.count(k)) v = kv[k]; };
    auto dbl = [&](const char* k, double& v) { if (kv.count(k)) v = std::stod(kv[k]); };
    auto num = [&](const char* k, int& v) { if (kv.count(k)) v = std::stoi(kv[k]); };
    str("--method", o.method); str("--map", o.map); str("--ply_dir", o.ply_dir);
    str("--init", o.init); str("--out", o.out); str("--stats", o.stats); str("--guess_file", o.guess_file);
    num("--max_frames", o.max_frames); dbl("--scale", o.scale); dbl("--source_voxel", o.source_voxel);
    dbl("--max_range", o.max_range); dbl("--epsilon", o.ndt_epsilon);
    dbl("--submap_half", o.submap_half); dbl("--recrop_dist", o.recrop_dist);
    dbl("--ndt_resolution", o.ndt_resolution); dbl("--ndt_step", o.ndt_step); num("--max_iter", o.max_iter);
    dbl("--gicp_max_corr", o.gicp_max_corr); num("--gicp_k", o.gicp_k); dbl("--reject_dist", o.reject_dist);
    dbl("--target_voxel", o.target_voxel);
    return !o.map.empty() && !o.ply_dir.empty() && (!o.init.empty() || !o.guess_file.empty()) && !o.out.empty() &&
           (o.method == "ndt" || o.method == "gicp");
}

std::vector<std::string> SortedPlyFiles(const std::string& dir) {
    std::vector<std::pair<long long, std::string>> files;
    for (const auto& e : std::filesystem::directory_iterator(dir)) {
        const std::string stem = e.path().stem().string();
        const auto pos = stem.rfind("_ts_");
        if (e.path().extension() != ".ply" || pos == std::string::npos) continue;
        files.emplace_back(std::stoll(stem.substr(pos + 4)), e.path().string());
    }
    std::sort(files.begin(), files.end());
    std::vector<std::string> paths;
    for (auto& f : files) paths.push_back(f.second);
    return paths;
}

Eigen::Matrix4f LoadInit(const std::string& path) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    std::ifstream in(path);
    std::string line;
    for (int r = 0; r < 4 && std::getline(in, line); ++r) {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);
        for (int c = 0; c < 4; ++c) ss >> T(r, c);
    }
    return T;
}

Cloud::Ptr Voxel(const Cloud::ConstPtr& in, double leaf) {
    auto out = std::make_shared<Cloud>();
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*out);
    return out;
}

Eigen::Matrix4f Orthonormalized(const Eigen::Matrix4f& T) {
    Eigen::Matrix4f out = T;
    Eigen::Quaternionf q(Eigen::Matrix3f(T.block<3, 3>(0, 0)));
    out.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    return out;
}

// 配準器的統一包裝：target 只在子地圖重切時更新，讓 G-ICP 的 target 協方差與 NDT 體素格可重複使用。
class Matcher {
public:
    explicit Matcher(const Options& o) : o_(o) {
        ndt_.setResolution(o.ndt_resolution);
        ndt_.setStepSize(o.ndt_step);
        ndt_.setTransformationEpsilon(o.ndt_epsilon);
        ndt_.setMaximumIterations(o.max_iter);
        gicp_.setMaxCorrespondenceDistance(o.gicp_max_corr);
        gicp_.setCorrespondenceRandomness(o.gicp_k);
        gicp_.setMaximumIterations(o.max_iter);
        gicp_.setTransformationEpsilon(1e-4);
        gicp_.setEuclideanFitnessEpsilon(1e-4);
    }

    void SetTarget(const Cloud::Ptr& target) {
        if (o_.method == "ndt") ndt_.setInputTarget(target);
        else gicp_.setInputTarget(target);
    }

    bool Align(const Cloud::Ptr& source, const Eigen::Matrix4f& guess, Eigen::Matrix4f& result, double& fitness) {
        Cloud aligned;
        if (o_.method == "ndt") {
            ndt_.setInputSource(source);
            ndt_.align(aligned, guess);
            result = ndt_.getFinalTransformation();
            fitness = ndt_.getFitnessScore(1.0);
            return ndt_.hasConverged();
        }
        gicp_.setInputSource(source);
        gicp_.align(aligned, guess);
        result = gicp_.getFinalTransformation();
        fitness = gicp_.getFitnessScore(1.0);
        return gicp_.hasConverged();
    }

private:
    const Options& o_;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;
};

Cloud::Ptr CropSubmap(const Cloud::Ptr& map, const Eigen::Vector3f& center, const Options& o) {
    auto submap = std::make_shared<Cloud>();
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(map);
    const float h = o.submap_half, hz = o.submap_half_z;
    crop.setMin(Eigen::Vector4f(center.x() - h, center.y() - h, center.z() - hz, 1.0f));
    crop.setMax(Eigen::Vector4f(center.x() + h, center.y() + h, center.z() + hz, 1.0f));
    crop.filter(*submap);
    return o.target_voxel > 0.0 ? Voxel(submap, o.target_voxel) : submap;
}

Cloud::Ptr LoadSource(const std::string& path, const Options& o) {
    auto raw = std::make_shared<Cloud>();
    pcl::io::loadPLYFile(path, *raw);
    auto scaled = std::make_shared<Cloud>();
    scaled->points.reserve(raw->size());
    for (auto p : raw->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        p.x *= o.scale; p.y *= o.scale; p.z *= o.scale;
        if (o.max_range > 0.0 && std::hypot(p.x, p.y) > o.max_range) continue;  // 與管線 FilterInputCloud 相同：縮放後的水平距離
        scaled->points.push_back(p);
    }
    scaled->width = static_cast<std::uint32_t>(scaled->size());
    scaled->height = 1;
    scaled->is_dense = true;
    return Voxel(scaled, o.source_voxel);
}

int RunGuessMode(const Options& o, const Cloud::Ptr& map, const std::vector<std::string>& plys) {
    std::ifstream in(o.guess_file);
    std::ofstream out(o.out);
    out.precision(9);
    Matcher matcher(o);
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        std::size_t idx;
        Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
        if (!(ss >> idx) || idx >= plys.size()) continue;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c) ss >> guess(r, c);
        matcher.SetTarget(CropSubmap(map, guess.block<3, 1>(0, 3), o));
        Eigen::Matrix4f result = guess;
        double fitness = -1.0;
        if (!matcher.Align(LoadSource(plys[idx], o), guess, result, fitness) || !result.allFinite()) result = guess;
        out << idx;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c) out << ' ' << result(r, c);
        out << ' ' << fitness << '\n';
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!ParseArgs(argc, argv, o)) {
        std::cerr << "參數不足，請見檔頭用法說明\n";
        return 1;
    }

    auto map = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile(o.map, *map) < 0 || map->empty()) {
        std::cerr << "讀取地圖失敗: " << o.map << "\n";
        return 1;
    }
    const auto plys = SortedPlyFiles(o.ply_dir);
    const std::size_t n = o.max_frames > 0 ? std::min<std::size_t>(o.max_frames, plys.size()) : plys.size();
    std::cerr << "map " << map->size() << " pts, frames " << n << ", method " << o.method << "\n";
    if (!o.guess_file.empty()) return RunGuessMode(o, map, plys);

    std::ofstream out(o.out);
    out.precision(9);
    std::ofstream stats;
    if (!o.stats.empty()) {
        stats.open(o.stats);
        stats << "idx,src_pts,tgt_pts,ms,converged,fitness,corr_m,corr_deg,rejected\n";
    }

    Matcher matcher(o);
    Eigen::Matrix4f pose = LoadInit(o.init);
    Eigen::Matrix4f prev_pose = pose;
    Eigen::Vector3f crop_center(1e9f, 1e9f, 1e9f);
    std::size_t target_pts = 0, rejected_total = 0;
    double total_ms = 0.0;

    for (std::size_t i = 0; i < n; ++i) {
        // 等速預測：沿用上一幀的相對運動
        const Eigen::Matrix4f guess = i < 2 ? pose : Orthonormalized(pose * (prev_pose.inverse() * pose));

        const Eigen::Vector3f center = guess.block<3, 1>(0, 3);
        if ((center - crop_center).norm() > o.recrop_dist) {
            const auto submap = CropSubmap(map, center, o);
            matcher.SetTarget(submap);
            target_pts = submap->size();
            crop_center = center;
        }

        const auto source = LoadSource(plys[i], o);

        const auto t0 = std::chrono::steady_clock::now();
        Eigen::Matrix4f result = guess;
        double fitness = -1.0;
        const bool converged = matcher.Align(source, guess, result, fitness);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        total_ms += ms;

        const Eigen::Matrix4f delta = guess.inverse() * result;
        const float corr_m = delta.block<3, 1>(0, 3).norm();
        const float corr_deg = Eigen::AngleAxisf(Eigen::Matrix3f(delta.block<3, 3>(0, 0))).angle() * 180.0f / M_PI;
        // 唯一的保護：明顯發散時沿用預測，兩種配準器共用相同門檻
        const bool reject = !converged || !result.allFinite() || (i > 0 && corr_m > o.reject_dist);
        rejected_total += reject;

        prev_pose = pose;
        pose = Orthonormalized(reject ? guess : result);

        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c) out << pose(r, c) << (r == 2 && c == 3 ? '\n' : ' ');
        if (stats.is_open()) {
            stats << i << ',' << source->size() << ',' << target_pts << ',' << ms << ',' << converged << ','
                  << fitness << ',' << corr_m << ',' << corr_deg << ',' << reject << '\n';
        }
        if (i % 100 == 0) {
            std::cerr << "frame " << i << " ms " << ms << " fit " << fitness << " corr " << corr_m << "m\n";
        }
    }
    std::cerr << "done: frames " << n << " avg_ms " << total_ms / std::max<std::size_t>(n, 1)
              << " rejected " << rejected_total << "\n";
    return 0;
}
