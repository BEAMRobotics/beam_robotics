#pragma once

// PCL
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <sensor_msgs/PointCloud2.h>

namespace lidar_util {

/*! Create a Fake point cloud for matching with detected point cloud
 * @param[in] line_dist - Vertical distance between lidar points
 * @param[out] point_dist - horizontal between lidar points
 * @param[in] side_length - length of side of square (diamond) board
 * @param[out] cloud - the resulant cloud
 *
 * Creates a fake point cloud mimicking the scan of the diamond-board
 */
void createFakePointCloud(double line_dist, double point_dist,
                          double side_length,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

/*! Computes normal distribution transform between two point clouds
 * @param[out] ndt - ndt object
 * @param[in] input_source - point cloud to be aligned
 * @param[in] input_target - point cloud to be aligned to
 * @param[in] initial_guess
 * @param[in] resolution - The Voxel Grid Covariance
 *
 * sets up and returns the ndt object. The ndt object can be used to get
 * properties like how well the scans matched up, and the final transformation
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
ndtMatch(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt,
         pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud,
         pcl::PointCloud<pcl::PointXYZ>::Ptr proper_generated_point_cloud,
         Eigen::Matrix4f initial_guess, double resolution);

/*! Fit a plane to input cloud, then project point cloud to that plane
 * @param[in] cloud - Input point cloud
 * @param[out] success - returns true if successfully fit plane
 *                  - False otherwise
 * @return projected point cloud
 * Runs ransac to fit plane to pointcloud then projects points onto that cloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
fitPlaneAndProject(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool &success);

/*! Filters the point cloud
 * @param[in] lidar_msg - Input point cloud
 * @param[in] T_Target_lidar - transformation to apply before filter
 *                           - Needs to be Matrix4f as transform point cloud
 *                             only accepts floats
 * @param[in] xy_limit - filters lidar points outside of +/-xy_limit
 * @param[in] z_limit - filters lidar points outside of +/-z_limit
 * Runs ransac to fit plane to pointcloud then projects points onto that cloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
filterPointCloud(const sensor_msgs::PointCloud2ConstPtr &lidar_msg,
                 Eigen::Matrix4f T_Target_lidar, double xy_limit,
                 double z_limit);
}
