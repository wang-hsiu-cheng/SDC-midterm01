#include "localization/coarse_to_fine_gicp.h"

namespace localization {

CoarseToFineGICP::CoarseToFineGICP(const CoarseToFineConfig& config)
    : config_(config) {}

pcl::PointCloud<pcl::PointXYZ>::Ptr CoarseToFineGICP::DownsampleCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, double leaf_size) const {
    if (leaf_size <= 0.0 || input->empty()) {
        auto copy = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        *copy = *input;
        return copy;
    }
    auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(input);
    const float leaf = static_cast<float>(leaf_size);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*output);
    return output;
}

bool CoarseToFineGICP::Align(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source_cloud,
                             const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target_submap,
                             const Eigen::Matrix4f& initial_guess,
                             Eigen::Matrix4f& final_pose,
                             double& final_score) {
    if (source_cloud->empty() || target_submap->empty()) {
        return false;
    }

    // ==========================================
    // Stage 1: Coarse Registration (粗配準 - 大體素)
    // ==========================================
    auto coarse_source = DownsampleCloud(source_cloud, config_.coarse.voxel_size);
    auto coarse_target = DownsampleCloud(target_submap, config_.coarse.voxel_size);

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_coarse;
    gicp_coarse.setTransformationEpsilon(config_.transformation_epsilon);
    gicp_coarse.setRotationEpsilon(config_.rotation_epsilon);
    gicp_coarse.setMaxCorrespondenceDistance(config_.coarse.max_correspondence_distance);
    gicp_coarse.setMaximumIterations(config_.coarse.max_iterations);
    gicp_coarse.setInputSource(coarse_source);
    gicp_coarse.setInputTarget(coarse_target);

    pcl::PointCloud<pcl::PointXYZ> coarse_aligned;
    gicp_coarse.align(coarse_aligned, initial_guess);

    Eigen::Matrix4f coarse_pose = initial_guess;
    if (gicp_coarse.hasConverged()) {
        coarse_pose = gicp_coarse.getFinalTransformation();
    }

    // ==========================================
    // Stage 2: Fine Registration (精配準 - 細體素)
    // ==========================================
    auto fine_source = DownsampleCloud(source_cloud, config_.fine.voxel_size);
    auto fine_target = DownsampleCloud(target_submap, config_.fine.voxel_size);

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_fine;
    gicp_fine.setTransformationEpsilon(config_.transformation_epsilon);
    gicp_fine.setRotationEpsilon(config_.rotation_epsilon);
    gicp_fine.setMaxCorrespondenceDistance(config_.fine.max_correspondence_distance);
    gicp_fine.setMaximumIterations(config_.fine.max_iterations);
    gicp_fine.setInputSource(fine_source);
    gicp_fine.setInputTarget(fine_target);

    pcl::PointCloud<pcl::PointXYZ> fine_aligned;
    gicp_fine.align(fine_aligned, coarse_pose);

    if (!gicp_fine.hasConverged()) {
        return false;
    }

    final_pose = gicp_fine.getFinalTransformation();
    final_score = gicp_fine.getFitnessScore();

    const bool is_finite = final_pose.array().isFinite().all() && std::isfinite(final_score);
    if (!is_finite) {
        return false;
    }

    // PCL 的 hasConverged() 或分數達到合格標準 (小於 max_acceptable_score) 即視為成功
    const bool converged = (gicp_fine.hasConverged() || final_score <= config_.max_acceptable_score);
    return converged;
}

}  // namespace localization