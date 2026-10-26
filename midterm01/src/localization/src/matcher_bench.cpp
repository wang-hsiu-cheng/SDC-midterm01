// 配準器比較基準：同一個最簡追蹤流程（等速預測 → 配準 → 寫出），只替換配準器，
// 用來判斷 NDT 與 G-ICP 何者適合作為重構後的核心。刻意不含任何限幅或濾波，
// 以免掩蓋配準器本身的差異。
//
// 用法：
//   matcher_bench --method ndt|gicp --map <pcd> --ply_dir <dir> --init <init_guess.txt>
//                 --out <result.txt> [--stats <stats.csv>] [--max_frames N] [--scale 8]
//                 [--source_voxel 0.5] [--max_range 0] [--epsilon 0.01] [--submap_half 50] [--recrop_dist 10]
//                 [--outlier_ratio 0.55] [--ndt_resolutions 4,3] [--source_min_z -1.2] [--keep_vertical 0.5] [--sor_k 20 --sor_std 1.0] [--source_side 1|-1]
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
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
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
    double outlier_ratio = -1.0;       // NDT 的離群比例（PCL 預設 0.55）；<0 = 用預設
    std::string ndt_resolutions;       // 粗到細，例如 "4,3"：依序對齊，前一階的結果當下一階的起點；空 = 只用 ndt_resolution
    double source_min_z = -1e9;        // 縮放後 z（向上）下限：去地面用，相機下方約 1.7 m 是地面
    double source_max_z = 1e9;
    double keep_vertical = 0.0;        // >0：只留法向 |n_z| < 此值的點（垂直面：牆、柱）；0 = 不篩
    double normal_radius = 1.5;        // m，法向估計半徑
    int source_side = 0;               // 1 = 只留左半（y>0）、-1 = 只留右半（y<0）：診斷點雲左右兩側各自把 NDT 拉向哪裡
    int sor_k = 0;                     // >0：統計式離群點去除的近鄰數
    double sor_std = 1.0;
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
    dbl("--ndt_resolution", o.ndt_resolution); dbl("--outlier_ratio", o.outlier_ratio); str("--ndt_resolutions", o.ndt_resolutions);
    dbl("--source_min_z", o.source_min_z); dbl("--source_max_z", o.source_max_z); dbl("--keep_vertical", o.keep_vertical);
    dbl("--normal_radius", o.normal_radius); num("--sor_k", o.sor_k); num("--source_side", o.source_side); dbl("--sor_std", o.sor_std); dbl("--ndt_step", o.ndt_step); num("--max_iter", o.max_iter);
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
        std::vector<double> resolutions;
        std::stringstream list(o.ndt_resolutions);
        for (std::string item; std::getline(list, item, ',');) {
            if (!item.empty()) resolutions.push_back(std::stod(item));
        }
        if (resolutions.empty()) resolutions.push_back(o.ndt_resolution);
        for (const double resolution : resolutions) {
            auto ndt = std::make_unique<Ndt>();
            ndt->setResolution(resolution);
            ndt->setStepSize(o.ndt_step);
            ndt->setTransformationEpsilon(o.ndt_epsilon);
            ndt->setMaximumIterations(o.max_iter);
            if (o.outlier_ratio >= 0.0) ndt->setOulierRatio(o.outlier_ratio);  // PCL 的函式名就是 setOulierRatio（拼字錯誤）;
            ndts_.push_back(std::move(ndt));
        }
        gicp_.setMaxCorrespondenceDistance(o.gicp_max_corr);
        gicp_.setCorrespondenceRandomness(o.gicp_k);
        gicp_.setMaximumIterations(o.max_iter);
        gicp_.setTransformationEpsilon(1e-4);
        gicp_.setEuclideanFitnessEpsilon(1e-4);
    }

    void SetTarget(const Cloud::Ptr& target) {
        if (o_.method == "ndt") {
            for (auto& ndt : ndts_) ndt->setInputTarget(target);
        } else {
            gicp_.setInputTarget(target);
        }
    }

    bool Align(const Cloud::Ptr& source, const Eigen::Matrix4f& guess, Eigen::Matrix4f& result, double& fitness) {
        Cloud aligned;
        if (o_.method == "ndt") {
            Eigen::Matrix4f current = guess;
            for (auto& ndt : ndts_) {  // 粗到細：前一階的結果當下一階的起點
                ndt->setInputSource(source);
                ndt->align(aligned, current);
                current = ndt->getFinalTransformation();
            }
            result = current;
            fitness = ndts_.back()->getFitnessScore(1.0);
            return ndts_.back()->hasConverged();
        }
        gicp_.setInputSource(source);
        gicp_.align(aligned, guess);
        result = gicp_.getFinalTransformation();
        fitness = gicp_.getFitnessScore(1.0);
        return gicp_.hasConverged();
    }

private:
    const Options& o_;
    using Ndt = pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
    std::vector<std::unique_ptr<Ndt>> ndts_;
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
        if (p.z < o.source_min_z || p.z > o.source_max_z) continue;
        if ((o.source_side > 0 && p.y <= 0.0) || (o.source_side < 0 && p.y >= 0.0)) continue;
        scaled->points.push_back(p);
    }
    scaled->width = static_cast<std::uint32_t>(scaled->size());
    scaled->height = 1;
    scaled->is_dense = true;
    Cloud::Ptr out = Voxel(scaled, o.source_voxel);
    if (o.keep_vertical > 0.0 && out->size() > 10) {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> estimation;
        pcl::PointCloud<pcl::Normal> normals;
        estimation.setInputCloud(out);
        estimation.setRadiusSearch(o.normal_radius);
        estimation.compute(normals);
        auto vertical = std::make_shared<Cloud>();
        for (std::size_t i = 0; i < out->size(); ++i) {
            if (std::isfinite(normals[i].normal_z) && std::abs(normals[i].normal_z) < o.keep_vertical) vertical->points.push_back(out->points[i]);
        }
        vertical->width = static_cast<std::uint32_t>(vertical->size());
        vertical->height = 1;
        vertical->is_dense = true;
        out = vertical;
    }
    if (o.sor_k > 0 && out->size() > static_cast<std::size_t>(o.sor_k)) {
        auto filtered = std::make_shared<Cloud>();
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(out);
        sor.setMeanK(o.sor_k);
        sor.setStddevMulThresh(o.sor_std);
        sor.filter(*filtered);
        out = filtered;
    }
    return out;
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
