#include <calibration_util/lidar_util.hpp>

#include <pcl/filters/passthrough.h>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <beam_utils/pcl_conversions.h>

namespace lidar_util {

void createFakePointCloud(double line_dist, double point_dist,
                          double side_length,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // center_to_corner_length is the distance from centre to outer end of target
  // Coordinate system: Origin at centre of target with x pointing down and y
  // pointing right
  double center_to_corner_length = side_length / sqrt(2);

  // Push back the end points for the target and for matching
  // Top point, right point, bottom point, left point
  cloud->points.push_back(pcl::PointXYZ(-center_to_corner_length, 0, 0));
  cloud->points.push_back(pcl::PointXYZ(0, center_to_corner_length, 0));
  cloud->points.push_back(pcl::PointXYZ(center_to_corner_length, 0, 0));
  cloud->points.push_back(pcl::PointXYZ(0, -center_to_corner_length, 0));

  // Loop over each row (separated by line_dist)
  // and column (separated by point_dist) to create the points within the
  // virtual pointcloud
  for (double x = 0; x < center_to_corner_length; x += line_dist) {
    double y_end = center_to_corner_length - x;

    if (x != 0) {
      // Add points along border if we are not on the horizontal (why? because
      // we added end points of horizontal above)
      cloud->points.push_back(pcl::PointXYZ(-x, y_end, 0));
      cloud->points.push_back(pcl::PointXYZ(x, y_end, 0));
      cloud->points.push_back(pcl::PointXYZ(x, -y_end, 0));
      cloud->points.push_back(pcl::PointXYZ(-x, -y_end, 0));
    }

    for (double y = 0; y < y_end; y += point_dist) {
      if (y == 0 && x == 0) {
        cloud->points.push_back(pcl::PointXYZ(0, 0, 0));
      } else if (y != 0 && x == 0) {
        cloud->points.push_back(pcl::PointXYZ(0, y, 0));
        cloud->points.push_back(pcl::PointXYZ(0, -y, 0));
      } else if (y == 0 && x != 0) {
        cloud->points.push_back(pcl::PointXYZ(x, 0, 0));
        cloud->points.push_back(pcl::PointXYZ(-x, 0, 0));
      } else if (y != 0 && x != 0) {
        cloud->points.push_back(pcl::PointXYZ(-x, y, 0));
        cloud->points.push_back(pcl::PointXYZ(x, y, 0));
        cloud->points.push_back(pcl::PointXYZ(x, -y, 0));
        cloud->points.push_back(pcl::PointXYZ(-x, -y, 0));
      }
    }
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
ndtMatch(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt,
         pcl::PointCloud<pcl::PointXYZ>::Ptr input_source,
         pcl::PointCloud<pcl::PointXYZ>::Ptr input_target,
         Eigen::Matrix4f initial_guess, double resolution) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr Finalndt(
      new pcl::PointCloud<pcl::PointXYZ>);
  ndt.setInputSource(input_source);
  ndt.setInputTarget(input_target);
  ndt.setMaximumIterations(200);
  ndt.setResolution(resolution);
  ndt.setTransformationEpsilon(0.0001);
  ndt.align(*Finalndt, initial_guess);
  return Finalndt;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
fitPlaneAndProject(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool &success) {
  // Fit plane to Segmented points
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    std::cout << "ERROR: Could not estimate a planar model for the given dataset.\n";
    success = false;
    return cloud;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud, *inliers, *plane_points);

  // Project points onto plane
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(plane_points);
  proj.setModelCoefficients(coefficients);
  proj.filter(*projected_cloud);
  success = true;
  return projected_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
filterPointCloud(const sensor_msgs::PointCloud2ConstPtr &lidar_msg,
                 Eigen::Matrix4f T_Target_lidar, double xy_limit,
                 double z_limit) {
  // Convert point cloud to PCL
  pcl::PCLPointCloud2 pcl_pc2;
  beam::pcl_conversions::toPCL(*lidar_msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
  // Change to camera frame
  pcl::transformPointCloud(*cloud, *cloud, T_Target_lidar);

  // Filter the point cloud
  // Filter z
  pcl::PassThrough<pcl::PointXYZ> passz;
  passz.setInputCloud(cloud);
  passz.setFilterFieldName("z");
  passz.setFilterLimits(-z_limit, z_limit);
  passz.filter(*cloud);

  // Filter x
  pcl::PassThrough<pcl::PointXYZ> passx;
  passx.setInputCloud(cloud);
  passx.setFilterFieldName("x");
  passx.setFilterLimits(-xy_limit, xy_limit);
  passx.filter(*cloud);
  // Filter y
  pcl::PassThrough<pcl::PointXYZ> passy;
  passy.setInputCloud(cloud);
  passy.setFilterFieldName("y");
  passy.setFilterLimits(-xy_limit, xy_limit);
  passy.filter(*cloud);

  pcl::transformPointCloud(*cloud, *cloud, T_Target_lidar.inverse());
  return cloud;
}

} // namespace lidar_util
