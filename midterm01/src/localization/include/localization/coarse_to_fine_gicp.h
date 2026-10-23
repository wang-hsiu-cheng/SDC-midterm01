#ifndef LOCALIZATION_COARSE_TO_FINE_GICP_H_
#define LOCALIZATION_COARSE_TO_FINE_GICP_H_

#include <memory>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>

namespace localization {

struct GICPLevelParams {
    double voxel_size;
    double max_correspondence_distance;
    int max_iterations;
};

struct CoarseToFineConfig {
    GICPLevelParams coarse{1.2, 3.0, 35};
    GICPLevelParams fine{0.3, 0.8, 25};
    double transformation_epsilon = 1e-4;
    double rotation_epsilon = 1e-4;
    double max_acceptable_score = 3.0;
};

class CoarseToFineGICP {
public:
    explicit CoarseToFineGICP(const CoarseToFineConfig& config = CoarseToFineConfig());

    // 執行兩階段 Coarse-to-Fine 配準
    bool Align(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source_cloud,
               const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target_submap,
               const Eigen::Matrix4f& initial_guess,
               Eigen::Matrix4f& final_pose,
               double& final_score);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr DownsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input, double leaf_size) const;

    CoarseToFineConfig config_;
};

}  // namespace localization

#endif  // LOCALIZATION_COARSE_TO_FINE_GICP_H_