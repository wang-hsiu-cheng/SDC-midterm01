#include <iostream>
#include <limits>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: scale_solver <path_to_ply> <path_to_map_pcd> [init_x init_y init_z yaw]\n";
        return -1;
    }

    std::string ply_file = argv[1];
    std::string map_file = argv[2];

    // 起點預設座標 (Track 1)
    float ix = (argc > 3) ? std::stof(argv[3]) : -4800.379f;
    float iy = (argc > 4) ? std::stof(argv[4]) : 2274.041f;
    float iz = (argc > 5) ? std::stof(argv[5]) : 109.928f;
    float iyaw = (argc > 6) ? std::stof(argv[6]) : 0.0f;

    std::cout << "[*] Loading raw PLY: " << ply_file << "\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPLYFile(ply_file, *raw_cloud) < 0) {
        std::cerr << "[-] Failed to load PLY!\n";
        return -1;
    }

    std::cout << "[*] Loading map PCD: " << map_file << "\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(map_file, *map_cloud) < 0) {
        std::cerr << "[-] Failed to load Map PCD!\n";
        return -1;
    }

    // 1. 裁剪起點附近的子地圖 (40x40m)
    pcl::PointCloud<pcl::PointXYZ>::Ptr submap(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(map_cloud);
    crop.setMin(Eigen::Vector4f(ix - 40.0f, iy - 40.0f, iz - 10.0f, 1.0f));
    crop.setMax(Eigen::Vector4f(ix + 40.0f, iy + 40.0f, iz + 10.0f, 1.0f));
    crop.filter(*submap);

    // 2. 子地圖降採樣提速
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(submap);
    voxel.setLeafSize(0.6f, 0.6f, 0.6f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr submap_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    voxel.filter(*submap_filtered);

    // 3. 設定初始位姿矩陣與相機仰角 (+0.07 rad)
    Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();
    init_pose(0, 0) = std::cos(iyaw); init_pose(0, 1) = -std::sin(iyaw);
    init_pose(1, 0) = std::sin(iyaw); init_pose(1, 1) =  std::cos(iyaw);
    init_pose(0, 3) = ix; init_pose(1, 3) = iy; init_pose(2, 3) = iz;

    Eigen::AngleAxisf pitch_rot(0.07f, Eigen::Vector3f::UnitY());
    Eigen::Matrix4f R_pitch = Eigen::Matrix4f::Identity();
    R_pitch.block<3, 3>(0, 0) = pitch_rot.toRotationMatrix();

    std::cout << "[*] 開始執行尺度搜尋 (Scale Sweep: 3.0 ~ 12.0) ...\n";
    double best_score = std::numeric_limits<double>::infinity();
    double best_scale = 3.0;

    // 4. 以 0.5 為間距掃描尺度 (共 19 個採樣點，幾秒鐘即可完成)
    for (double s = 3.0; s <= 12.0; s += 0.5) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr scaled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        scaled_cloud->points.reserve(raw_cloud->size());

        for (const auto& pt : raw_cloud->points) {
            if (!pcl::isFinite(pt)) continue;
            Eigen::Vector3f p(s * pt.x, s * pt.y, s * pt.z);
            p = pitch_rot * p;
            scaled_cloud->points.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
        }

        // 輸入點雲降採樣
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        voxel.setInputCloud(scaled_cloud);
        voxel.setLeafSize(0.6f, 0.6f, 0.6f);
        voxel.filter(*input_filtered);

        // 執行快速 G-ICP 檢驗幾何適配殘差
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(input_filtered);
        gicp.setInputTarget(submap_filtered);
        gicp.setMaxCorrespondenceDistance(2.0);
        gicp.setMaximumIterations(15);
        gicp.setTransformationEpsilon(1e-3);

        pcl::PointCloud<pcl::PointXYZ> aligned;
        gicp.align(aligned, init_pose);

        double score = gicp.getFitnessScore();
        std::cout << "  Scale = " << s << " -> Fitness Score = " << score << "\n";

        if (gicp.hasConverged() && score < best_score) {
            best_score = score;
            best_scale = s;
        }
    }

    std::cout << "\n=======================================================\n";
    std::cout << "  [+] 最佳尺度 (Best ply_scale) = " << best_scale << " (Score: " << best_score << ")\n";
    std::cout << "=======================================================\n";
    return 0;
}