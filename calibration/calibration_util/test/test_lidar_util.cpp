#include "calibration_util/lidar_util.hpp"

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include <math.h>
#include <stdlib.h>

#include <gtest/gtest.h>

#include <cmath>
const auto PI = M_PI;

namespace lidar_util {

/*! Tests filterPointCloud
 * 1. Confirms that the points outside the specified limits are filtered
 * and that none of the points inside the specified region are filtered.
 */
TEST(TestLidarUtil, Test_filterPointCloud) {
  const double kLimitXY = 2.3;
  const double kLimitZ = 1.5;

  pcl::PointCloud<pcl::PointXYZ>::Ptr inside_limits(
      new pcl::PointCloud<pcl::PointXYZ>);

  srand(time(NULL));
  double x, y, z;
  // Points that should not be filtered
  for (int i = 0; i < 100; i++) {
    x = ((double)rand() / RAND_MAX) * 2 * kLimitXY - kLimitXY;
    y = ((double)rand() / RAND_MAX) * 2 * kLimitXY - kLimitXY;
    z = ((double)rand() / RAND_MAX) * 2 * kLimitZ - kLimitZ;
    inside_limits->points.push_back(pcl::PointXYZ(x, y, z));
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr full_point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*inside_limits, *full_point_cloud);

  // Points that should be filtered
  for (int i = 0; i < 50; i++) {
    while (abs(x) <= kLimitXY) {
      x = ((double)rand() / RAND_MAX) * 10 * kLimitXY - 5 * kLimitXY;
    }
    while (abs(y) <= kLimitXY) {
      y = ((double)rand() / RAND_MAX) * 10 * kLimitXY - 5 * kLimitXY;
    }
    while (abs(z) <= kLimitZ) {
      z = ((double)rand() / RAND_MAX) * 10 * kLimitZ - 5 * kLimitZ;
    }
    full_point_cloud->points.push_back(pcl::PointXYZ(x, y, z));
  }
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(*full_point_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 sens_message;
  pcl_conversions::fromPCL(pcl_pc2, sens_message);
  const sensor_msgs::PointCloud2ConstPtr &input =
      std::make_shared<sensor_msgs::PointCloud2>(sens_message);
  Eigen::Matrix4f eye = Eigen::Matrix4f::Identity();
  // Filter points
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered =
      filterPointCloud(input, eye, kLimitXY, kLimitZ);
  EXPECT_EQ(inside_limits->size(), filtered->size());
  for (int i = 0; i < 100; i++) {
    EXPECT_EQ(inside_limits->points[i].x, filtered->points[i].x);
    EXPECT_EQ(inside_limits->points[i].y, filtered->points[i].y);
    EXPECT_EQ(inside_limits->points[i].z, filtered->points[i].z);
  }
}

/*! Tests fitPlaneAndProject
 * 1. Tests that the function filters out points well off of the plane
 * 2. Tests that the function can fitPlane to reasonable test case
 * 3. Tests that the function keeps only the points on the plane and all the
 * points on the plane.
 */
TEST(TestLidarUtil, Test_fitPlaneAndProject) {
  // create points on plane
  const double kLimitXY = 10.0;
  const double kRangeZ = 10.0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  srand(time(NULL));
  // Points already on plane
  double x, y, z;
  const int kNumPointsOnPlane = 100;
  for (int i = 0; i < kNumPointsOnPlane; i++) {
    x = ((double)rand() / RAND_MAX) * 2 * kLimitXY - kLimitXY;
    y = ((double)rand() / RAND_MAX) * 2 * kLimitXY - kLimitXY;
    cloud->points.push_back(pcl::PointXYZ(x, y, 0));
  }

  // Create points off of plane
  const int kNumPointsOffPlane = 5;
  for (int i = 0; i < kNumPointsOffPlane; i++) {
    x = ((double)rand() / RAND_MAX) * 2 * kLimitXY - kLimitXY;
    y = ((double)rand() / RAND_MAX) * 2 * kLimitXY - kLimitXY;
    z = ((double)rand() / RAND_MAX) * kRangeZ + 0.1;
    if (i % 2 == 0) {
      z = -z;
    }
    cloud->points.push_back(pcl::PointXYZ(x, y, z));
  }

  bool success = false;
  pcl::PointCloud<pcl::PointXYZ>::Ptr result =
      fitPlaneAndProject(cloud, success);
  EXPECT_TRUE(success);
  EXPECT_EQ(kNumPointsOnPlane, result->size());
  for (int i = 0; i < result->size(); i++) {
    EXPECT_LT(fabs(result->points[i].z), 0.0001);
  }
}

/*! Tests ndtMatch
 * 1. Tests that ndtMatch can, for the cases specified below:
 *   a: Converge
 *   b: Match with score less than 0.1
 *   b: Return the correct transforms
 * Cases:
 * 1.1 Small x, y, and z displacement (each axis tested independently)
 * 1.2 Large x displacement
 * 1.3 Small rotation about x, y, and z axis (each axis tested independently)
 * 1.4 Large rotation about x, y, and z axis (each axis tested independently)
 * 1.5 All large rotations together
 * 1.6 All small displacements together
 * 1.7 All small rotations then small displacements
 * 1.8 All small displacements then small rotations
 */
TEST(TestLidarUtil, Test_ndtMatch) {
  const bool kDebug = false;
  // Create base point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  const double kOuterSqLen = 0.56;
  const double kLineSpaceing = .02;
  const double kPointSpaceing = .005;
  createFakePointCloud(kLineSpaceing, kPointSpaceing, kOuterSqLen, input_cloud);

  Eigen::Matrix4f init_guess;
  // Turning off clang format so matrix stays as 4x4 and not in one line
  // clang-format off
  init_guess << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
  const int kNumberOfTests = 13;
  Eigen::Matrix4d transform[kNumberOfTests];

  // Small x displacement
  const double kSmallDisp = 0.05;
  transform[0] << 1, 0, 0, kSmallDisp,
                  0, 1, 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1;

  // Small y displacement
  transform[1] << 1, 0, 0, 0,
                  0, 1, 0, kSmallDisp,
                  0, 0, 1, 0,
                  0, 0, 0, 1;

  // Small z displacement
  transform[2] << 1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, kSmallDisp,
                  0, 0, 0, 1;

  // larger x displacement
  transform[3] << 1, 0, 0, 1,
                  0, 1, 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1;

  // small_rot about x
  const double kSmallAngle = 2 * PI / 180.0;
  transform[3] << 1, 0, 0, 0,
                  0, cos(kSmallAngle), -sin(kSmallAngle), 0,
                  0, sin(kSmallAngle), cos(kSmallAngle), 0,
                  0, 0, 0, 1;

  // small_rot about y
  transform[4] << cos(kSmallAngle), 0, -sin(kSmallAngle), 0,
                  0, 1, 0, 0,
                  sin(kSmallAngle), 0, cos(kSmallAngle),  0,
                  0, 0, 0, 1;

  // small rot about z
  transform[5] << 1, 0, 0, 0,
                  0, cos(kSmallAngle), -sin(kSmallAngle), 0,
                  0, sin(kSmallAngle), cos(kSmallAngle), 0,
                  0, 0, 0, 1;


  const double kLargeAngle = 20 * PI /180.0;
  // large_rot about x
  transform[6] << 1, 0, 0, 0,
                  0, cos(kLargeAngle), -sin(kLargeAngle), 0,
                  0, sin(kLargeAngle), cos(kLargeAngle),  0,
                  0, 0, 0, 1;

  // large_rot about y
  transform[7] << cos(kLargeAngle), 0, -sin(kLargeAngle), 0,
                  0, 1, 0, 0,
                  sin(kLargeAngle), 0, cos(kLargeAngle),  0,
                  0, 0, 0, 1;

  // large_rot about z
  transform[8] << cos(kLargeAngle), -sin(kLargeAngle), 0, 0,
                  sin(kLargeAngle), cos(kLargeAngle), 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1;
  // clang-format on

  // All large rotations
  transform[9] = transform[8] * transform[7] * transform[6];

  // All translations
  transform[10] = transform[0] * transform[1] * transform[2];

  // rotations then translations
  transform[11] = transform[10] * transform[9];

  // translations then rotations
  transform[12] = transform[9] * transform[10];

  for (int trans_num = 0; trans_num < kNumberOfTests; trans_num++) {
    // Get target cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*input_cloud, *target_cloud, transform[trans_num]);

    // Run ndt match
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    const double kResolution = 0.5;
    pcl::PointCloud<pcl::PointXYZ>::Ptr result =
        ndtMatch(ndt, target_cloud, input_cloud, init_guess, kResolution);
    Eigen::Matrix4f result_transform = ndt.getFinalTransformation().inverse();

    if (kDebug) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*input_cloud, *output_cloud, result_transform);
      std::cout << "ndt.getFitnessScore()" << ndt.getFitnessScore() << '\n';
      std::cout << "original transform:\n" << transform[trans_num] << '\n';
      std::cout << "Result transform:\n" << result_transform << '\n';

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr concatenated_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>());
      concatenated_cloud->points.resize(output_cloud->points.size() +
                                        target_cloud->points.size());
      int j = -1;
      for (int i = 0; i < output_cloud->points.size(); i++, j++) {
        concatenated_cloud->points[j].x = output_cloud->points[i].x;
        concatenated_cloud->points[j].y = output_cloud->points[i].y;
        concatenated_cloud->points[j].z = output_cloud->points[i].z;
        concatenated_cloud->points[j].r = 0;
        concatenated_cloud->points[j].g = 255;
        concatenated_cloud->points[j].b = 0;
      }
      for (int i = 0; i < target_cloud->points.size(); i++, j++) {
        concatenated_cloud->points[j].x = target_cloud->points[i].x;
        concatenated_cloud->points[j].y = target_cloud->points[i].y;
        concatenated_cloud->points[j].z = target_cloud->points[i].z;
        concatenated_cloud->points[j].r = 255;
        concatenated_cloud->points[j].g = 0;
        concatenated_cloud->points[j].b = 0;
      }
      std::cout << "-------------" << std::endl;
      pcl::visualization::CloudViewer viewer("Cloud Viewer");
      viewer.showCloud(concatenated_cloud);
      while (!viewer.wasStopped()) {
      }
    }

    EXPECT_TRUE(ndt.hasConverged());
    EXPECT_LT(ndt.getFitnessScore(), 0.1);

    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        EXPECT_LT(fabs(result_transform(i, j) - transform[trans_num](i, j)),
                  0.01);
      }
    }
  }
}

} // lidar_util
