#include "localization/road_heading_prior.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace localization {

namespace {

constexpr double kRad2Deg = 180.0 / M_PI;

// 5 點二項式模糊（零邊界），沿某一軸
void Blur(std::vector<double>& img, int n, bool along_x) {
    constexpr std::array<double, 5> k{1.0 / 16, 4.0 / 16, 6.0 / 16, 4.0 / 16, 1.0 / 16};
    std::vector<double> out(img.size(), 0.0);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            double sum = 0.0;
            for (int t = -2; t <= 2; ++t) {
                const int ii = along_x ? i + t : i, jj = along_x ? j : j + t;
                if (ii < 0 || ii >= n || jj < 0 || jj >= n) continue;
                sum += k[t + 2] * img[ii * n + jj];
            }
            out[i * n + j] = sum;
        }
    }
    img.swap(out);
}

}  // namespace

RoadHeadingPrior::Result RoadHeadingPrior::Estimate(const pcl::PointCloud<pcl::PointXYZ>& map,
                                                    const Eigen::Matrix4f& init_pose) const {
    Result result;
    const double cx = init_pose(0, 3), cy = init_pose(1, 3), cz = init_pose(2, 3);
    const int n = static_cast<int>(2.0 * config_.radius / config_.cell);
    std::vector<double> img(static_cast<std::size_t>(n) * n, 0.0);

    for (const auto& p : map.points) {
        if (std::abs(p.x - cx) >= config_.radius || std::abs(p.y - cy) >= config_.radius) continue;
        if (p.z <= cz + config_.z_min || p.z >= cz + config_.z_max) continue;
        const int i = static_cast<int>((p.x - (cx - config_.radius)) / config_.cell);
        const int j = static_cast<int>((p.y - (cy - config_.radius)) / config_.cell);
        if (i < 0 || i >= n || j < 0 || j >= n) continue;
        img[i * n + j] += 1.0;
        ++result.points;
    }
    if (result.points < 200) return result;

    for (auto& v : img) v = std::min(v, 5.0);  // 抑制單格過密
    Blur(img, n, true);
    Blur(img, n, false);

    // 梯度（內部中央差分、邊界單邊差分）→ 邊緣方向 = 梯度方向 + 90°，以梯度平方加權做 180 格直方圖
    std::array<double, 180> hist{};
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            const double gx = (i == 0) ? img[(i + 1) * n + j] - img[i * n + j]
                              : (i == n - 1) ? img[i * n + j] - img[(i - 1) * n + j]
                                             : 0.5 * (img[(i + 1) * n + j] - img[(i - 1) * n + j]);
            const double gy = (j == 0) ? img[i * n + j + 1] - img[i * n + j]
                              : (j == n - 1) ? img[i * n + j] - img[i * n + j - 1]
                                             : 0.5 * (img[i * n + j + 1] - img[i * n + j - 1]);
            const double weight = gx * gx + gy * gy;
            if (weight <= 0.0) continue;
            double theta = std::fmod(std::atan2(gy, gx) * kRad2Deg + 90.0 + 360.0, 180.0);
            hist[std::min(179, static_cast<int>(theta))] += weight;
        }
    }
    std::array<double, 180> smooth{};
    for (int b = 0; b < 180; ++b) {
        double sum = 0.0;
        for (int t = -2; t <= 2; ++t) sum += hist[(b + t + 180) % 180];
        smooth[b] = sum / 5.0;
    }
    result.wall_deg = static_cast<double>(std::max_element(smooth.begin(), smooth.end()) - smooth.begin()) + 0.5;

    // 解 180° 模糊：取與 init_guess 的 yaw 最近者
    const double init_yaw_deg = std::atan2(init_pose(1, 0), init_pose(0, 0)) * kRad2Deg;
    const auto wrap = [](double a) { return std::fmod(std::fmod(a + 180.0, 360.0) + 360.0, 360.0) - 180.0; };
    double best = result.wall_deg, best_dev = wrap(result.wall_deg - init_yaw_deg);
    const double other_dev = wrap(result.wall_deg + 180.0 - init_yaw_deg);
    if (std::abs(other_dev) < std::abs(best_dev)) {
        best = result.wall_deg + 180.0;
        best_dev = other_dev;
    }
    result.deviation_deg = best_dev;
    if (std::abs(best_dev) > config_.max_deviation_deg) return result;  // 離 init_guess 太遠：這個起點的牆面方向不可靠
    result.yaw_rad = wrap(best) / kRad2Deg;
    result.found = true;
    return result;
}

}  // namespace localization
