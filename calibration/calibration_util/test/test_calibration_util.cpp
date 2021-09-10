#include <calibration_util/calibration_util.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <ros/package.h>
#include <gtest/gtest.h>

namespace calibration_util {

// Tests generateChessboardCorners
// Checks that x, y and z points of corners are accurate
TEST(TestCalibrationUtil, Test_GenerateChessboardCorners) {
  cv::Mat corners;
  int width = 2;
  int height = 3;
  cv::Size chessboard_size(width, height); // Width 2 height 3
  double squareSize = 1.2;
  double offset_x = 0.5;
  double offset_y = 0.1;

  generateChessboardCorners(chessboard_size, squareSize, corners, offset_x,
                            offset_y);
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      EXPECT_EQ(corners.at<double>(i * width + j, 0),
                i * squareSize + offset_x);
      EXPECT_EQ(corners.at<double>(i * width + j, 1),
                j * squareSize + offset_y);
      EXPECT_EQ(corners.at<double>(i * width + j, 2), 0);
    }
  }
}

// Tests readCameraMatrix
// 1. Checks if false value is returned if improper path is sent
// 2. Checks if data can be read and is accurate
TEST(TestCalibrationUtil, Test_readCameraMatrix) {
  // Unable to read
  cv::Mat cameraMatrix, distCoeffs;
  EXPECT_FALSE(readCameraMatrix("a", cameraMatrix, distCoeffs));

  std::string path = ros::package::getPath("calibration_util");
  EXPECT_TRUE(readCameraMatrix(path + "/test_data/read_camera_matrix.yaml",
                               cameraMatrix, distCoeffs));
  int i;
  double k;
  for (i = 0, k = 0.0; i < 3; i++) {
    for (int j = 0; j < 3; j++, k++) {
      EXPECT_DOUBLE_EQ(cameraMatrix.at<double>(i, j), k / 10.0);
    }
  }
  for (i = 0; i < 5; i++) {
    EXPECT_EQ(distCoeffs.at<double>(i), i + 1);
  }
}

// Tests getRMSerror
// 1. Ensure error is sent if sizes are different
// 2. Test all zeros
// 3. Test simple case - all values are 1
// 4. Test simple case but with doubles
// 5. Test case input 2 and 14
// 6. Test case input 8,9,8,4
// 7. Test case same as prev but transposed
TEST(TestCalibrationUtil, Test_RMSE) {
  // Test unequal matricies
  cv::Mat unequal1 = cv::Mat::zeros(2, 3, CV_64FC1);
  cv::Mat unequal2 = cv::Mat::zeros(2, 4, CV_64FC1);
  cv::Mat unequal3 = cv::Mat::zeros(3, 3, CV_64FC1);
  EXPECT_EQ(getRMSerror(unequal1, unequal2), -1);
  EXPECT_EQ(getRMSerror(unequal1, unequal3), -1);

  // Test matrix with non-two value
  EXPECT_EQ(getRMSerror(unequal3, unequal3), -1);

  // Test improper size
  cv::Mat improper_size = cv::Mat::zeros(2, 2, CV_64FC2);
  EXPECT_EQ(getRMSerror(improper_size, improper_size), -1);

  // Test zero
  EXPECT_EQ(getRMSerror(unequal2, unequal2), 0);

  // Test simple case
  cv::Mat zero_mat = cv::Mat::zeros(2, 2, CV_64FC1);
  double d_one[2][2] = {{1, 0}, {0, 1}};
  cv::Mat one_mat = cv::Mat(2, 2, CV_64FC1, d_one);
  EXPECT_EQ(getRMSerror(one_mat, zero_mat), 1);

  // Test simple case with double
  zero_mat = cv::Mat::zeros(2, 3, CV_64FC1);
  double temp_value = 2.3;
  double d_temp[3][2] = {{temp_value, 0}, {temp_value, 0}, {temp_value, 0}};
  cv::Mat error_mat = cv::Mat(2, 3, CV_64FC1, d_temp);
  EXPECT_EQ(getRMSerror(error_mat, zero_mat), temp_value);

  // Test slightly more complicated case
  zero_mat = cv::Mat::zeros(2, 2, CV_64FC1);
  double d_2_14[2][2] = {{2, 14}, {0, 0}};
  cv::Mat mat_2_14 = cv::Mat(2, 2, CV_64FC1, d_2_14);
  EXPECT_EQ(getRMSerror(zero_mat, mat_2_14), 10);

  // Most complex case
  cv::Mat one_mat_4 = cv::Mat::ones(2, 4, CV_64FC1);
  double d_4[4][2] = {{9, 1}, {10, 1}, {1, 9}, {1, 5}};
  cv::Mat mat_4 = cv::Mat(2, 4, CV_64FC1, d_4);
  EXPECT_EQ(getRMSerror(one_mat_4, mat_4), 7.5);

  // Most complex case transposed
  cv::Mat one_mat_4t = cv::Mat::ones(4, 2, CV_64FC1);
  double d_4t[2][4] = {{9, 1, 10, 1}, {1, 9, 1, 5}};
  cv::Mat mat_4t = cv::Mat(4, 2, CV_64FC1, d_4t);
  EXPECT_EQ(getRMSerror(one_mat_4t, mat_4t), 7.5);

  // Test 2 channels
  cv::Mat two_chan_ones;
  std::vector<cv::Mat> comb_ones;
  comb_ones.push_back(cv::Mat::ones(4, 1, CV_64FC1));
  comb_ones.push_back(cv::Mat::ones(4, 1, CV_64FC1));
  cv::merge(comb_ones, two_chan_ones);

  double first_channel[4] = {9, 1, 10, 1};
  double second_channel[4] = {1, 9, 1, 5};
  cv::Mat two_chan = cv::Mat::ones(4, 1, CV_64FC2);
  std::vector<cv::Mat> comb_channels;
  comb_channels.push_back(cv::Mat(4, 1, CV_64FC1, first_channel));
  comb_channels.push_back(cv::Mat(4, 1, CV_64FC1, second_channel));
  cv::merge(comb_channels, two_chan);

  EXPECT_EQ(getRMSerror(two_chan, two_chan_ones), 7.5);

  // Test two channels transposed
  cv::Mat two_chan_ones_t;
  std::vector<cv::Mat> comb_ones_t;
  comb_ones_t.push_back(cv::Mat::ones(1, 4, CV_64FC1));
  comb_ones_t.push_back(cv::Mat::ones(1, 4, CV_64FC1));
  cv::merge(comb_ones_t, two_chan_ones_t);

  cv::Mat two_chan_t = cv::Mat::ones(1, 4, CV_64FC2);
  std::vector<cv::Mat> comb_channels_t;
  comb_channels_t.push_back(cv::Mat(1, 4, CV_64FC1, first_channel));
  comb_channels_t.push_back(cv::Mat(1, 4, CV_64FC1, second_channel));
  cv::merge(comb_channels_t, two_chan_t);

  EXPECT_EQ(getRMSerror(two_chan_t, two_chan_ones_t), 7.5);
}

// Tests readTransform
// Tests that transform can be properly read
TEST(TestCalibrationUtil, Test_readTransform) {
  // Unable to read
  Eigen::Matrix4d a;
  EXPECT_FALSE(readTransform("a", a));

  std::string path = ros::package::getPath("calibration_util");
  EXPECT_TRUE(readTransform(path + "/test_data/readTransform.txt", a));
  int i;
  double k;
  for (i = 0, k = 0.0; i < 4; i++) {
    for (int j = 0; j < 4; j++, k += 0.1) {
      EXPECT_DOUBLE_EQ(a(i, j), k);
    }
  }
}

// Tests combineRotationAndTranslationMat
// Starts with a projection matrix
// Decompose the matrix
// Confirm that re-assembled matrix matches original matrix
TEST(TestCalibrationUtil, Test_combineRotationAndTranslationMat) {
  for (int test_num = 0; test_num < 3; test_num++) {
    cv::Mat P = cv::Mat::eye(3, 4, CV_64FC1);
    // Check no rotation and 1 translation in z
    if (test_num == 0) {
      P.at<double>(0, 0) = 0;
      P.at<double>(1, 0) = -1;
      P.at<double>(2, 0) = 0;

      P.at<double>(0, 1) = 1;
      P.at<double>(1, 1) = 0;
      P.at<double>(2, 1) = 0;

      P.at<double>(0, 2) = 0;
      P.at<double>(1, 2) = 0;
      P.at<double>(2, 2) = 1;

      P.at<double>(0, 3) = 0;
      P.at<double>(1, 3) = 0;
      P.at<double>(2, 3) = 1;
    }
    // Different P where K is close to eye
    if (test_num == 1) {
      P.at<double>(0, 0) = 0.01590973768769244;
      P.at<double>(1, 0) = 0.999606501542073;
      P.at<double>(2, 0) = 0.02310243107394315;

      P.at<double>(0, 1) = 0.9981780701437231;
      P.at<double>(1, 1) = -0.014533583628767;
      P.at<double>(2, 1) = -0.05856035545535897;

      P.at<double>(0, 2) = -0.05820155093175065;
      P.at<double>(1, 2) = 0.02399201995920976;
      P.at<double>(2, 2) = -0.998016514115581;

      P.at<double>(0, 3) = -0.04917548212454651;
      P.at<double>(1, 3) = -0.6349460444222967;
      P.at<double>(2, 3) = 2.387392077722901;
    }
    // Different P with non K eye
    if (test_num == 2) {
      P.at<double>(0, 0) = -2.8058e-01;
      P.at<double>(1, 0) = -6.8326e-02;
      P.at<double>(2, 0) = 5.1458e-07;

      P.at<double>(0, 1) = 2.0045e-02;
      P.at<double>(1, 1) = -3.1718e-01;
      P.at<double>(2, 1) = 4.5840e-06;

      P.at<double>(0, 2) = 1.8102e-01;
      P.at<double>(1, 2) = -7.2974e-02;
      P.at<double>(2, 2) = 2.6699e-06;

      P.at<double>(0, 3) = 6.6062e-01;
      P.at<double>(1, 3) = 5.8402e-01;
      P.at<double>(2, 3) = 1.5590e-03;
    }

    cv::Mat K, rvec, T_homogeneous; // translation vector

    cv::decomposeProjectionMatrix(P, K, rvec, T_homogeneous);
    cv::Mat T(3, 1, CV_64FC1); // translation vector

    cv::convertPointsFromHomogeneous(T_homogeneous.reshape(4, 1), T);
    T = T.reshape(1, 3);

    cv::Mat tr_test;
    combineRotationAndTranslationMat(tr_test, rvec, -1 * rvec * T);

    tr_test(cv::Range(0, tr_test.cols - 1), cv::Range(0, tr_test.rows))
        .copyTo(tr_test);
    cv::Mat P_mat = K * tr_test;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
        EXPECT_TRUE(
            abs(P_mat.at<double>(i, j) - P.at<double>(i, j)) <=
            std::min(abs(P_mat.at<double>(i, j)), abs(P.at<double>(i, j))) /
                100.0);
      }
    }
  }
}

// Test type2str
// Tests all options
TEST(TestCalibrationUtil, Test_type2str) {
  EXPECT_EQ(type2str(CV_8UC1), "CV_8UC1");
  EXPECT_EQ(type2str(CV_8UC2), "CV_8UC2");
  EXPECT_EQ(type2str(CV_8UC3), "CV_8UC3");
  EXPECT_EQ(type2str(CV_8UC4), "CV_8UC4");

  EXPECT_EQ(type2str(CV_8SC1), "CV_8SC1");
  EXPECT_EQ(type2str(CV_8SC2), "CV_8SC2");
  EXPECT_EQ(type2str(CV_8SC3), "CV_8SC3");
  EXPECT_EQ(type2str(CV_8SC4), "CV_8SC4");

  EXPECT_EQ(type2str(CV_16UC1), "CV_16UC1");
  EXPECT_EQ(type2str(CV_16UC2), "CV_16UC2");
  EXPECT_EQ(type2str(CV_16UC3), "CV_16UC3");
  EXPECT_EQ(type2str(CV_16UC4), "CV_16UC4");

  EXPECT_EQ(type2str(CV_16SC1), "CV_16SC1");
  EXPECT_EQ(type2str(CV_16SC2), "CV_16SC2");
  EXPECT_EQ(type2str(CV_16SC3), "CV_16SC3");
  EXPECT_EQ(type2str(CV_16SC4), "CV_16SC4");

  EXPECT_EQ(type2str(CV_32SC1), "CV_32SC1");
  EXPECT_EQ(type2str(CV_32SC2), "CV_32SC2");
  EXPECT_EQ(type2str(CV_32SC3), "CV_32SC3");
  EXPECT_EQ(type2str(CV_32SC4), "CV_32SC4");

  EXPECT_EQ(type2str(CV_32FC1), "CV_32FC1");
  EXPECT_EQ(type2str(CV_32FC2), "CV_32FC2");
  EXPECT_EQ(type2str(CV_32FC3), "CV_32FC3");
  EXPECT_EQ(type2str(CV_32FC4), "CV_32FC4");

  EXPECT_EQ(type2str(CV_64FC1), "CV_64FC1");
  EXPECT_EQ(type2str(CV_64FC2), "CV_64FC2");
  EXPECT_EQ(type2str(CV_64FC3), "CV_64FC3");
  EXPECT_EQ(type2str(CV_64FC4), "CV_64FC4");
}

// Test splitString
// 1. No delimiter
// 2. delimiter in middle
// 3. only delimeter
// 4. delimeter at edges
TEST(TestCalibrationUtil, Test_splitString) {
  // No delimiter
  std::vector<std::string> a = {"a"};
  EXPECT_EQ(splitString("a", "b"), a);

  // delimiter in middle
  a = {"a", "c"};
  EXPECT_EQ(splitString("abc", "b"), a);

  // multiple delimiters in middle
  a = {"a", "cd", "e"};
  EXPECT_EQ(splitString("abcdbe", "b"), a);

  // only delimiter
  a = {};
  EXPECT_EQ(splitString("b", "b"), a);

  // only delimiters
  a = {};
  EXPECT_EQ(splitString("bbbbb", "b"), a);

  // delimiter at end
  a = {"a"};
  EXPECT_EQ(splitString("ba", "b"), a);

  // delimiter at beggining
  a = {"a"};
  EXPECT_EQ(splitString("ab", "b"), a);
}

// Tests readTransformList
// 1. Checks that error is returned if invalid filename
// 2. Checks transoform is read and data is set properly, this also tests
//    makeStampedTransform implicitly
TEST(TestCalibrationUtil, Test_readTransformList) {
  std::vector<std::pair<std::string, std::string>> names;
  std::vector<Eigen::Matrix4d> extrinsic_transformations_list;
  std::vector<tf::StampedTransform> extrinsic_transformations_stamped_list;
  int count = 4;

  std::string name_ans[4][2] = {{"velodyne", "GPS"},
                                {"CAMERA", "velodyne"},
                                {"FOO", "BAR"},
                                {"TIC", "TAC"}};

  tf::Matrix3x3 eye;
  eye.setIdentity();

  Eigen::Matrix4d m;
  m << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  ASSERT_FALSE(readTransformList("a", names, extrinsic_transformations_list,
                                 extrinsic_transformations_stamped_list,
                                 count));
  std::string path = ros::package::getPath("calibration_util") +
                     "/test_data/readTransformList_test.yaml";
  ASSERT_TRUE(readTransformList(path, names, extrinsic_transformations_list,
                                extrinsic_transformations_stamped_list, count));

  for (int i = 0; i < count; i++) {
    EXPECT_EQ(name_ans[i][1], std::get<0>(names[i]));
    EXPECT_EQ(name_ans[i][0], std::get<1>(names[i]));
    EXPECT_EQ(m, extrinsic_transformations_list[i]);
    if (i != count - 1) // Dont test last one
    {
      EXPECT_EQ(name_ans[i][0],
                extrinsic_transformations_stamped_list[i].frame_id_);
      EXPECT_EQ(name_ans[i][1],
                extrinsic_transformations_stamped_list[i].child_frame_id_);
      EXPECT_EQ(extrinsic_transformations_stamped_list[i].getBasis(), eye);
    }
  }
  EXPECT_EQ(names.size(), extrinsic_transformations_stamped_list.size() + 1);
}

// Tests readListOfTransforms
// 1. Checks that error is returned if invalid filename
// 2. Checks transoform is read and data is set properly, this also tests
//    makeStampedTransform implicitly
TEST(TestCalibrationUtil, Test_readListOfTransforms) {
  std::vector<std::string> names;
  std::vector<Eigen::Matrix4d> extrinsic_transformations_list;
  std::vector<tf::StampedTransform> extrinsic_transformations_stamped_list;
  int count = 3;

  std::string name_ans[3] = {"F", "LB", "imuRB"};

  tf::Matrix3x3 eye;
  eye.setIdentity();

  Eigen::Matrix4d m;
  m << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  ASSERT_FALSE(readListOfTransforms("a", names, extrinsic_transformations_list,
                                    extrinsic_transformations_stamped_list));
  std::string path = ros::package::getPath("calibration_util") +
                     "/test_data/extrinsics/listOfTransforms.txt";
  ASSERT_TRUE(readListOfTransforms(path, names, extrinsic_transformations_list,
                                   extrinsic_transformations_stamped_list));
  std::string lidar = "velodyne";
  for (int i = 0; i < count; i++) {
    EXPECT_EQ(name_ans[i], names[i]);
    EXPECT_EQ(m, extrinsic_transformations_list[i]);
    EXPECT_EQ(lidar, extrinsic_transformations_stamped_list[i].frame_id_);
    EXPECT_EQ(name_ans[i],
              extrinsic_transformations_stamped_list[i].child_frame_id_);
    EXPECT_EQ(extrinsic_transformations_stamped_list[i].getBasis(), eye);
  }
}

// Tests getExtrinsicTransform
// 1. Check false returned if pair not found
// 2. Check true and data is set if pair is valid
TEST(TestCalibrationUtil, Test_getExtrinsicTransform) {
  std::pair<std::string, std::string> search_for, append;
  std::vector<std::pair<std::string, std::string>> have;
  have.push_back(std::make_pair("a", "b"));
  have.push_back(std::make_pair("b", "c"));
  have.push_back(std::make_pair("c", "a"));

  Eigen::Matrix4d m, result;
  m << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  result << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  std::vector<Eigen::Matrix4d> transforms;
  transforms.assign(3, m);

  search_for = std::make_pair("b", "a");
  ASSERT_FALSE(getExtrinsicTransform(search_for, result, have, transforms));
  ASSERT_FALSE(result.isApprox(m));
  search_for = std::make_pair("a", "b");
  EXPECT_TRUE(getExtrinsicTransform(search_for, result, have, transforms));
  EXPECT_TRUE(result.isApprox(m));
}

// Tests both PoinXYZToMat and MatToPoinXYZ
// 1. Start with mat, convert to point and then convert back. original matrix
//    and final matrix are equal
TEST(TestCalibrationUtil, Test_MatAndPCLConversion) {
  cv::Mat a = cv::Mat::eye(5, 3, CV_64FC1);
  a.at<double>(3, 0) = 0.32;
  a.at<double>(3, 1) = 12.3;
  a.at<double>(3, 2) = 1.85;

  pcl::PointCloud<pcl::PointXYZ>::Ptr b = MatToPoinXYZ(a);
  cv::Mat c = PoinXYZToMat(b);
  for (int i = 0; i < a.rows; i++) {
    for (int j = 0; j < a.cols; j++) {
      EXPECT_NEAR(a.at<double>(i, j), c.at<double>(i, j), 0.00001);
    }
  }
}

// Tests both writeMeasurements and readTextFile
// 1. Tests can write to text file
// 2. Tests can read from file
// 3. Tests that the data read is same as data written
TEST(TestCalibrationUtil, TEST_WriteAndReadMeasurements) {
  std::string filename = ros::package::getPath("calibration_util") +
                         "/test_data/measurement_test.txt";
  std::vector<std::vector<double>> measurements;
  std::vector<double> inner = {1.2, 3.4, 4.5, 5.6, 6.7};
  measurements.push_back(inner);
  inner.assign(5, 1);
  measurements.push_back(inner);
  inner.assign(5, 9.87654321);
  measurements.push_back(inner);
  EXPECT_TRUE(writeMeasurements(filename, measurements));

  std::vector<std::vector<double>> recv_measurements;
  EXPECT_FALSE(readTextFile("a", recv_measurements, 3, 5));
  EXPECT_TRUE(readTextFile(filename, recv_measurements, 3, 5));
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 5; j++) {
      EXPECT_NEAR(measurements[i][j], recv_measurements[i][j], 0.0001);
    }
  }
}

} // calibration_util
