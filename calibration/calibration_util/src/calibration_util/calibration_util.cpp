#include "calibration_util/calibration_util.hpp"

#include <iostream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Geometry>

namespace calibration_util {
// Get chessboard corners
void generateChessboardCorners(cv::Size boardSize, double squareSize,
                               cv::Mat &corners, double offset_x,
                               double offset_y) {
  for (int i = 0; i < boardSize.height; i++)
    for (int j = 0; j < boardSize.width; j++) {
      cv::Mat row =
          (cv::Mat_<double>(1, 3) << double(i * squareSize + offset_x),
           double(j * squareSize + offset_y), double(0.0));
      corners.push_back(row);
    }
}

// Read Intrinsic Camera Matrix and distortion coefficients
bool readCameraMatrix(const std::string &filename, cv::Mat &cameraMatrix,
                      cv::Mat &distCoeffs) {
  struct stat buffer;
  if (stat(filename.c_str(), &buffer) != 0)
    return false;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  cv::FileNode fn_cammat = fs["camera_matrix"];
  cv::FileNode fn_dist = fs["distortion_coefficients"];

  std::vector<double> cam_mat;
  fn_cammat["data"] >> cam_mat;
  cameraMatrix =
      (cv::Mat_<double>(3, 3) << cam_mat[0], cam_mat[1], cam_mat[2], cam_mat[3],
       cam_mat[4], cam_mat[5], cam_mat[6], cam_mat[7], cam_mat[8]);

  std::vector<double> distortions;
  fn_dist["data"] >> distortions;
  if (distortions.size() == 5) {
    distCoeffs = (cv::Mat_<double>(5, 1) << distortions[0], distortions[1],
                  distortions[2], distortions[3], distortions[4]);
  } else if (distortions.size() == 4) {
    distCoeffs = (cv::Mat_<double>(4, 1) << distortions[0], distortions[1],
                  distortions[2], distortions[3]);
  }
  if (distCoeffs.type() != CV_64F)
    distCoeffs = cv::Mat_<double>(distCoeffs);
  if (cameraMatrix.type() != CV_64F)
    cameraMatrix = cv::Mat_<double>(cameraMatrix);

  return true;
}

// Calculate RMS Error
double getRMSerror(const cv::Mat &projected_pts,
                   const cv::Mat &detected_corners) {

  if ((projected_pts.rows != detected_corners.rows) |
      (projected_pts.cols != detected_corners.cols) |
      (projected_pts.channels() != detected_corners.channels())) {
    std::cout << "RMS ERROR num rows, cols or channels not equal" << std::endl;
    return -1;
  } else if ((projected_pts.rows != 2) & (projected_pts.cols != 2) &
             (projected_pts.channels() != 2)) {
    std::cout << "RMS ERROR num rows, cols or channels not equal to 2\n";
    return -1;
  } else if ((projected_pts.rows != 1) & (projected_pts.cols != 1) &
             (projected_pts.channels() != 1)) {
    std::cout << "RMS ERROR num rows, cols or channels not equal to 1\n";
    return -1;
  }
  bool multi_channel = (projected_pts.channels() == 2);
  double error_sum = 0.0;
  for (int i = 0; i < detected_corners.rows; i++) {
    for (int j = 0; j < detected_corners.cols; j++) {
      if (multi_channel) {
        for (int k = 0; k < detected_corners.channels(); k++) {
          error_sum += pow(projected_pts.at<cv::Vec3d>(i, j)[k] -
                               detected_corners.at<cv::Vec3d>(i, j)[k],
                           2);
        }
      } else {
        error_sum += pow(projected_pts.at<double>(i, j) -
                             detected_corners.at<double>(i, j),
                         2);
      }
    }
  }
  // We only want to divide by number of points, which is either #row or #cols
  // Since one of either rows or cols  or channels is equal to 2, we multiply by
  // 2.
  // one of the other values, either rows or cols  or channels, is equal to 1
  // Therefore the devisor below is equal to number of points
  return sqrt(error_sum / detected_corners.rows / detected_corners.cols /
              detected_corners.channels() * 2);
}

bool readTransform(std::string filename, Eigen::Matrix4d &eig_mat) {
  // Reading final transform
  std::ifstream fin;
  fin.open(filename.c_str());

  if (!fin) {
    std::cout << "Could not open file" << std::endl;
    return 0;
  }

  double a;

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      fin >> a;
      eig_mat(i, j) = a;
    }
  }

  fin.close();
  return 1;
}

// Set Projection Matrix
// This does not take into consideration camera projection????
void combineRotationAndTranslationMat(cv::Mat &P_mat, const cv::Mat &R_mat,
                                      const cv::Mat &t_vec) {
  cv::hconcat(R_mat, t_vec, P_mat);
  if (type2str(R_mat.type()) == "CV_64FC1") {
    cv::Mat row = (cv::Mat_<double>(1, 4) << double(0.0), double(0.0),
                   double(0.0), double(1.0));
    P_mat.push_back(row);
  } else if (type2str(R_mat.type()) == "CV_32FC1") {
    cv::Mat row = (cv::Mat_<float>(1, 4) << float(0.0), float(0.0), float(0.0),
                   float(1.0));
    P_mat.push_back(row);
  } else {
    throw std::invalid_argument("R_mat is not defined as 64FC1 or 32FC1.");
  }
}

// Get type of OpenCV matrix
std::string type2str(int type) {
  std::string r;
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);
  switch (depth) {
  case CV_8U:
    r = "CV_8U";
    break;
  case CV_8S:
    r = "CV_8S";
    break;
  case CV_16U:
    r = "CV_16U";
    break;
  case CV_16S:
    r = "CV_16S";
    break;
  case CV_32S:
    r = "CV_32S";
    break;
  case CV_32F:
    r = "CV_32F";
    break;
  case CV_64F:
    r = "CV_64F";
    break;
  default:
    r = "User";
    break;
  }
  r += "C";
  r += (chans + '0');
  return r;
}

// TODO: Nicholash: replac with m.convertTo(m, CV_32FC1)
// Convert OpenCV matrix to 32FC1
void convertTo32FC1(const cv::Mat &from, cv::Mat &to, int rows, int cols) {
  for (int r = 0; r < rows; r++)
    for (int c = 0; c < cols; c++) {
      to.at<float>(r, c) = from.at<float>(r, c);
    }
}

std::vector<std::string> splitString(std::string original_string,
                                     std::string delimiter) {
  size_t pos = 0;
  std::string token;
  std::vector<std::string> split_strings;
  while ((pos = original_string.find(delimiter)) != std::string::npos) {
    token = original_string.substr(0, pos);
    original_string.erase(0, pos + delimiter.length());
    if (token != "")
      split_strings.push_back(token);
  }
  if (original_string != "")
    split_strings.push_back(original_string);
  return split_strings;
}

tf::StampedTransform makeStampedTransform(Eigen::Matrix4d &Tm,
                                          std::string from_frame,
                                          std::string to_frame) {
  tf::Vector3 origin;
  tf::Matrix3x3 tf3d;

  origin.setValue(static_cast<double>(Tm(0, 3)), static_cast<double>(Tm(1, 3)),
                  static_cast<double>(Tm(2, 3)));
  tf3d.setValue(static_cast<double>(Tm(0, 0)), static_cast<double>(Tm(0, 1)),
                static_cast<double>(Tm(0, 2)), static_cast<double>(Tm(1, 0)),
                static_cast<double>(Tm(1, 1)), static_cast<double>(Tm(1, 2)),
                static_cast<double>(Tm(2, 0)), static_cast<double>(Tm(2, 1)),
                static_cast<double>(Tm(2, 2)));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(tfqt);
  return tf::StampedTransform(transform, ros::Time(0), to_frame, from_frame);
}

bool readTransformList(std::string transformation_list,
                       std::vector<std::pair<std::string, std::string>> &names,
                       std::vector<Eigen::Matrix4d> &transformations,
                       std::vector<tf::StampedTransform> &stamped_transforms,
                       double NO_OF_TRANSFORMS) {

  // Reading transformation list text file to get names of transforms
  std::ifstream fin;
  std::cout << transformation_list << std::endl;
  fin.open(transformation_list.c_str());
  cv::FileStorage fs(transformation_list, cv::FileStorage::READ);

  if (!fin) {
    std::cout << "Could not open transformation list file: "
              << transformation_list.c_str() << std::endl;
    return false;
  }

  std::string transformation_name;
  int count = 1;
  Eigen::Matrix4d transform_mat;
  std::string from_frame;
  std::string to_frame;

  while (std::getline(fin, transformation_name)) {
    // Since first line in yaml file is %YAML:=1.0
    if (count > 1) {

      std::vector<std::string> split_strings =
          splitString(transformation_name, ":");
      std::vector<std::string> tr_name = splitString(split_strings[0], "_");

      from_frame = tr_name[2].compare("LIDAR") == 0 ? "velodyne" : tr_name[2];
      to_frame = tr_name[1].compare("LIDAR") == 0 ? "velodyne" : tr_name[1];

      std::vector<double> tr_mat;
      fs[split_strings[0]] >> tr_mat;

      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
          transform_mat(i, j) = tr_mat[4 * i + j];
        }
      }

      // Store the names (from_frame, to_frame)
      names.push_back(std::make_pair(from_frame, to_frame));

      // Store the transformations
      transformations.push_back(transform_mat);

      // Store stamped transformations
      // Dont take the last transform as it will create a loop and cause
      // problems
      if (count < NO_OF_TRANSFORMS + 1)
        stamped_transforms.push_back(
            makeStampedTransform(transform_mat, from_frame, to_frame));
    }
    count++;
  }

  return true;
}

bool readListOfTransforms(
    std::string transformation_list, std::vector<std::string> &names,
    std::vector<Eigen::Matrix4d> &transformations,
    std::vector<tf::StampedTransform> &stamped_transforms) {

  // Reading transformation list text file to get names of transforms
  std::ifstream fin;
  fin.open(transformation_list.c_str());

  if (!fin) {
    std::cout << "Could not open transformation list file" << std::endl;
    return false;
  }

  std::string transform_name;
  Eigen::Matrix4d transform_mat;
  Eigen::Matrix4d Tm;

  // Concatenate transform name to path
  std::vector<std::string> split_strings =
      splitString(transformation_list, "/");
  std::string concatenated_name = "/";
  for (unsigned int i = 0; i < split_strings.size() - 1; i++) {
    concatenated_name += split_strings[i] + "/";
  }

  // Read the transform name
  while (fin >> transform_name) {
    std::string transformation_path = concatenated_name + transform_name;
    if (readTransform(transformation_path, transform_mat)) {

      // Lidar to camera transform
      transformations.push_back(transform_mat);

      // Estimates for visualization purposes
      Tm = transform_mat;

      // Names of Cameras
      std::vector<std::string> split_strings = splitString(transform_name, "_");
      names.push_back(split_strings[0]);

      stamped_transforms.push_back(
          makeStampedTransform(Tm, split_strings[0], "velodyne"));
    } else {
      std::cout << concatenated_name << std::endl;
    }
  }
  return true;
}

bool getExtrinsicTransform(
    std::pair<std::string, std::string> &frame_pair,
    Eigen::Matrix4d &extrinsic_transform,
    std::vector<std::pair<std::string, std::string>> &names,
    std::vector<Eigen::Matrix4d> &transformations) {

  // Get frames from transformation
  std::string from_frame = frame_pair.first;
  std::string to_frame = frame_pair.second;

  for (unsigned int i = 0; i < transformations.size(); i++) {
    if (from_frame.compare(names[i].first) == 0 &&
        to_frame.compare(names[i].second) == 0) {
      extrinsic_transform = transformations[i];
      return 1;
    }
  }
  return 0; // Get chessboard corners
}

void drawBothChessboardCorners(cv::Mat &image, cv::Size pattern_size,
                               cv::Mat corners_base, cv::Mat corners_proj)

{
  cv::Scalar base_colour(0, 255, 0);
  cv::Scalar link_colour(0, 200, 200);
  cv::Scalar proj_colour(0, 0, 255);
  int i, x, y;
  int radius = 5;
  int line_type = CV_AA;
  for (y = 0, i = 0; y < pattern_size.height; y++) {
    for (x = 0; x < pattern_size.width; x++, i++) {
      cv::Point pt_base, pt_proj;
      pt_base.x = cvRound(corners_base.at<double>(i, 0));
      pt_base.y = cvRound(corners_base.at<double>(i, 1));
      pt_proj.x = cvRound(corners_proj.at<double>(i, 0));
      pt_proj.y = cvRound(corners_proj.at<double>(i, 1));
      // Draw base circle
      cv::circle(image, pt_base, radius, base_colour, 1, line_type);

      // Draw projected crosshair
      cv::line(image, cv::Point(pt_proj.x + radius, pt_proj.y),
               cv::Point(pt_proj.x - radius, pt_proj.y), proj_colour, 1,
               line_type);
      cv::line(image, cv::Point(pt_proj.x, pt_proj.y + radius),
               cvPoint(pt_proj.x, pt_proj.y - radius), proj_colour, 1,
               line_type);
      // Draw link between points
      cv::line(image, pt_base, pt_proj, link_colour, 1, line_type);
    }
  }
  // Draw legend
  cv::putText(image, "Points from this camera",
              cv::Point2f(int(2 * image.cols / 3.0) - 30, 50),
              cv::FONT_HERSHEY_PLAIN, 2, base_colour, 2);
  cv::putText(image, "Points from other camera",
              cv::Point2f(int(2 * image.cols / 3.0) - 30, 90),
              cv::FONT_HERSHEY_PLAIN, 2, proj_colour, 2);
}

cv::Mat PoinXYZToMat(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr) {
  cv::Mat OpenCVPointCloud(point_cloud_ptr->points.size(), 3, CV_64FC1);
  for (unsigned int i = 0; i < point_cloud_ptr->points.size(); i++) {
    OpenCVPointCloud.at<double>(i, 0) = point_cloud_ptr->points[i].x;
    OpenCVPointCloud.at<double>(i, 1) = point_cloud_ptr->points[i].y;
    OpenCVPointCloud.at<double>(i, 2) = point_cloud_ptr->points[i].z;
  }
  return OpenCVPointCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(cv::Mat OpencVPointCloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>); //(new
                                           // pcl::pointcloud<pcl::pointXYZ>);

  std::cout << "num cols " << OpencVPointCloud.cols << "\n";
  for (int i = 0; i < OpencVPointCloud.rows; i++) {
    pcl::PointXYZ point;
    point.x = OpencVPointCloud.at<double>(i, 0);
    point.y = OpencVPointCloud.at<double>(i, 1);
    point.z = OpencVPointCloud.at<double>(i, 2);
    point_cloud_ptr->points.push_back(point);
  }
  point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
  point_cloud_ptr->height = 1;

  return point_cloud_ptr;
}

// Sort Target points to preserve order
void sortCentres(std::vector<cv::Point> const original_centres,
                 std::vector<cv::Point> &new_centres,
                 cv::Point &target_centre_point) {
  float mean_x = 0.0, mean_y = 0.0;
  for (unsigned int i = 0; i < original_centres.size(); i++) {
    mean_x += original_centres[i].x;
    mean_y += original_centres[i].y;
    new_centres.push_back(cv::Point(0, 0));
  }
  mean_x = mean_x / original_centres.size();
  mean_y = mean_y / original_centres.size();

  target_centre_point.x = mean_x;
  target_centre_point.y = mean_y;

  for (unsigned int i = 0; i < original_centres.size(); i++) {
    float ptx = original_centres[i].x;
    float pty = original_centres[i].y;
    float diffx = ptx - target_centre_point.x;
    float diffy = pty - target_centre_point.y;
    if (abs(diffx) > abs(diffy)) { // circle is either 1 or 3
      if (diffx > 0) {
        new_centres[1].x = ptx;
        new_centres[1].y = pty;
      } else {
        new_centres[3].x = ptx;
        new_centres[3].y = pty;
      }
    } else { // circle is either 0 or 2
      if (diffy <
          0) { // Since y pixel values increase as we move down in the image
        new_centres[0].x = ptx;
        new_centres[0].y = pty;
      } else {
        new_centres[2].x = ptx;
        new_centres[2].y = pty;
      }
    }
  }
}

void generateTargetCorners(float dist, cv::Mat &corners) {

  // Since target is a diamond, assume coordinate system in middle with x
  // pointing down and y pointing right
  // top
  corners.at<float>(0, 0) = float(-dist);
  corners.at<float>(0, 1) = float(0);
  corners.at<float>(0, 2) = float(0);

  // right
  corners.at<float>(1, 0) = float(0);
  corners.at<float>(1, 1) = float(dist);
  corners.at<float>(1, 2) = float(0);

  // bottom
  corners.at<float>(2, 0) = float(dist);
  corners.at<float>(2, 1) = float(0);
  corners.at<float>(2, 2) = float(0);

  // left
  corners.at<float>(3, 0) = float(0);
  corners.at<float>(3, 1) = float(-dist);
  corners.at<float>(3, 2) = float(0);
}
//

// TODO: Nicholash Replace with Eigen::Map function
void matOpencvToEigen(const cv::Mat &opencvMat, Eigen::Matrix4f &eigenMat) {
  eigenMat.resize(opencvMat.rows, opencvMat.cols);
  for (int i = 0; i < opencvMat.rows; i++)
    for (int j = 0; j < opencvMat.cols; j++)
      eigenMat(i, j) = opencvMat.at<float>(i, j);
}

bool writeMeasurements(std::string filename,
                       std::vector<std::vector<double>> measurements) {
  int num_meas_size = measurements.size();
  int each_meas_size = measurements[0].size();

  std::ofstream fout;
  fout.open(filename.c_str());

  if (!fout) {
    std::cout << "Could not open file" << std::endl;
    return 0;
  }

  for (int i = 0; i < num_meas_size; i++) {
    for (int j = 0; j < each_meas_size; j++) {
      fout << measurements[i][j] << " ";
    }
    fout << std::endl;
  }

  fout.close();
  return 1;
}

bool readTextFile(std::string filename,
                  std::vector<std::vector<double>> &measurement_pts,
                  int num_meas, int meas_size) {
  std::ifstream fin;
  fin.open(filename.c_str());

  if (!fin) {
    std::cout << "Could not open file" << std::endl;
    return 0;
  }
  for (int i = 0; i < num_meas; i++) {
    std::vector<double> measurement;
    for (int j = 0; j < meas_size; j++) {
      double a;
      fin >> a;
      measurement.push_back(a);
    }
    measurement_pts.push_back(measurement);
  }
  return 1;
}

} // namespace calibration_util
