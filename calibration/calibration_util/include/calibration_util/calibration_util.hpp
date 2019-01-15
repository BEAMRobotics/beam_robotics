#ifndef CALIBRATION_UTIL_HPP
#define CALIBRATION_UTIL_HPP

#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/core/core.hpp>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/image_encodings.h>

namespace calibration_util {

// Calculate chessboard corners
void generateChessboardCorners(cv::Size, double, cv::Mat &, double, double);

// Read Intrinsic Camera Matrix
bool readCameraMatrix(const std::string &, cv::Mat &, cv::Mat &);

// Get RMS error
double getRMSerror(const cv::Mat &, const cv::Mat &);

// Read transformation from file
bool readTransform(std::string, Eigen::Matrix4d &);

// Set the projection matrix
void combineRotationAndTranslationMat(cv::Mat &, const cv::Mat &,
                                      const cv::Mat &);

// Determine type of Opencv mat
std::string type2str(int);

// Convert Matrix to 32FC1
void convertTo32FC1(const cv::Mat &, cv::Mat &, int, int);

// Read List of Transforms
bool readTransformList(std::string,
                       std::vector<std::pair<std::string, std::string>> &,
                       std::vector<Eigen::Matrix4d> &,
                       std::vector<tf::StampedTransform> &, double);

bool readListOfTransforms(
    std::string transformation_list, std::vector<std::string> &,
    std::vector<Eigen::Matrix4d> &transformations,
    std::vector<tf::StampedTransform> &stamped_transforms);

// Create StampedTransform
tf::StampedTransform makeStampedTransform(Eigen::Matrix4d &, std::string,
                                          std::string);

// Split String based on delimeter
std::vector<std::string> splitString(std::string, std::string);

// Get Extrinsic Transform
bool getExtrinsicTransform(std::pair<std::string, std::string> &,
                           Eigen::Matrix4d &,
                           std::vector<std::pair<std::string, std::string>> &,
                           std::vector<Eigen::Matrix4d> &);

void drawBothChessboardCorners(cv::Mat &, cv::Size, const cv::Mat,
                               const cv::Mat);

cv::Mat PoinXYZToMat(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr);

pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(cv::Mat OpencVPointCloud);

void sortCentres(std::vector<cv::Point> const, std::vector<cv::Point> &,
                 cv::Point &);

void generateTargetCorners(float, cv::Mat &);

void createFakePointCloud(double, double, double,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr);

void matOpencvToEigen(const cv::Mat &, Eigen::Matrix4f &);

bool writeMeasurements(std::string, std::vector<std::vector<double>>);

bool readTextFile(std::string, std::vector<std::vector<double>> &, int, int);
}

#endif // CALIBRATION_UTIL_HPP
