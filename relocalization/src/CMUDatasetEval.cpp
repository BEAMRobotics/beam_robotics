#include <ImageDatabase.h>
#include <ImageToImage.h>
#include <beam_calibration/CameraModels.h>
#include <beam_cv/descriptors/Descriptors.h>
#include <beam_cv/detectors/Detectors.h>
#include <beam_cv/geometry/AbsolutePoseEstimator.h>
#include <beam_cv/matchers/Matchers.h>
#include <beam_utils/utils.hpp>
#include <boost/filesystem.hpp>
#include <chrono>
#include <nlohmann/json.hpp>

// variables
std::shared_ptr<beam_calibration::CameraModel> c0;
std::shared_ptr<beam_calibration::CameraModel> c1;

std::shared_ptr<beam_cv::Descriptor> desc =
    std::make_shared<beam_cv::ORBDescriptor>();
std::shared_ptr<beam_cv::Detector> det =
    std::make_shared<beam_cv::ORBDetector>();
std::shared_ptr<beam_cv::Matcher> mat =
    std::make_shared<beam_cv::FLANNMatcher>();
std::string cmu_path;
std::string slice_num;
relocalization::ImageToImage matcher;

using namespace std;
using namespace std::chrono;

// functions

void tokenize(std::string const& str, const char delim,
              std::vector<std::string>& out) {
  size_t start;
  size_t end = 0;

  while ((start = str.find_first_not_of(delim, end)) != std::string::npos) {
    end = str.find(delim, start);
    out.push_back(str.substr(start, end - start));
  }
}

int GetNumberLines(std::string file) {
  int number_of_lines = 0;
  std::string line;
  std::ifstream myfile(file);

  while (std::getline(myfile, line)) ++number_of_lines;
  return number_of_lines;
}

void EvaluateQueryImages() {
  // declare variables
  char delim = ' ';
  std::ifstream infile;
  std::ofstream outfile;
  std::string line;
  std::string query_txt_file = cmu_path + "/slice" + slice_num +
                               "/test-images-slice" + slice_num + ".txt";
  std::string output_txt_file = cmu_path + "/slice" + slice_num +
                                "/test-images-slice" + slice_num + "output.txt";
  std::string query_images_folder = cmu_path + "/slice" + slice_num + "/query";
  int lines = GetNumberLines(query_txt_file);
  int cur_line = 0;
  infile.open(query_txt_file);
  outfile.open(output_txt_file);
  // extract poses
  while (!infile.eof()) {
    if (cur_line % 10 == 0) {
      BEAM_INFO("Processing image #{} out of {}", cur_line, lines);
    }
    cur_line++;
    std::getline(infile, line, '\n');
    if (line.size() == 0) { break; }
    std::vector<std::string> out;
    tokenize(line, delim, out);
    std::string image_file = out[0];
    std::string image_path = query_images_folder + "/" + image_file;
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    std::shared_ptr<beam_calibration::CameraModel> cam_model;
    std::string cam_model_str = image_file.substr(10, 2);
    if (cam_model_str == "c0") {
      cam_model = c0;
    } else {
      cam_model = c1;
    }
    std::string no_match = image_file + " 0 0 0 0 0 0 0\n";
    auto start = high_resolution_clock::now();
    std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector3d>> correspondences =
        matcher.Query(image);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    cout << "Matcher duration: " << duration.count() << " ms" << endl;
    // write zeros if no image match found
    if (correspondences.size() == 0) {
      outfile << no_match;
      continue;
    }
    std::vector<Eigen::Vector2i> pixels;
    std::vector<Eigen::Vector3d> points;
    for (auto& corr : correspondences) {
      pixels.push_back(std::get<0>(corr));
      points.push_back(std::get<1>(corr));
    }
    start = high_resolution_clock::now();
    Eigen::Matrix4d P = beam_cv::AbsolutePoseEstimator::RANSACEstimator(
        cam_model, pixels, points, 100, 10.0, time(0));
    stop = high_resolution_clock::now();
    duration = duration_cast<microseconds>(stop - start);
    cout << "P3P duration: " << duration.count() << " ms" << endl;
    Eigen::Matrix3d R = P.block<3, 3>(0, 0);
    Eigen::Vector3d t = P.block<3, 1>(0, 3);
    Eigen::Quaterniond Rq(R);
    std::string quat_string =
        std::to_string(Rq.w()) + " " + std::to_string(Rq.x()) + " " +
        std::to_string(Rq.y()) + " " + std::to_string(Rq.z());
    std::string trans_string = std::to_string(t[0]) + " " +
                               std::to_string(t[1]) + " " +
                               std::to_string(t[2]);
    std::string line_to_write =
        image_file + " " + quat_string + " " + trans_string + "\n";
    outfile << line_to_write;
  }
  infile.close();
  outfile.close();
}

void LoadCameraModels() {
  std::ifstream intrinsics_txt;
  std::string intrinsics_path = cmu_path + "/intrinsics.txt";
  intrinsics_txt.open(intrinsics_path);
  std::string c0_line, c1_line, s;
  // skip first 24 lines
  for (int i = 0; i < 24; ++i) std::getline(intrinsics_txt, s);

  std::getline(intrinsics_txt, c0_line);
  std::vector<std::string> out;
  tokenize(c0_line, ' ', out);
  int width = atof(out[2].c_str());
  int height = atof(out[3].c_str());
  out.erase(out.begin(), out.begin() + 4);
  std::vector<double> intrinsics;
  for (auto i : out) { intrinsics.push_back(atof(i.c_str())); }
  Eigen::Matrix<double, 8, 1> intrinsics_c0(intrinsics.data());
  c0 = std::make_shared<beam_calibration::Radtan>(height, width, intrinsics_c0);
  c0->WriteJSON(cmu_path + "/c0.json");

  std::getline(intrinsics_txt, c1_line);
  std::vector<std::string> out2;
  tokenize(c1_line, ' ', out2);
  out2.erase(out2.begin(), out2.begin() + 4);
  intrinsics.clear();
  for (auto i : out2) { intrinsics.push_back(atof(i.c_str())); }
  Eigen::Matrix<double, 8, 1> intrinsics_c1(intrinsics.data());
  c1 = std::make_shared<beam_calibration::Radtan>(height, width, intrinsics_c1);
  c1->WriteJSON(cmu_path + "/c1.json");
}

std::shared_ptr<relocalization::ImageDatabase>
    BuildDB(const std::string& db_path, const std::string& vocab_path) {
  std::string image_folder = db_path + "/database";
  std::shared_ptr<relocalization::ImageDatabase> id =
      std::make_shared<relocalization::ImageDatabase>(vocab_path, desc, det,
                                                      db_path, image_folder);
  // declare variables
  char delim = ' ';
  std::ifstream infile;
  std::string line;
  std::string GT_file_path =
      db_path + "/ground-truth-database-images-slice" + slice_num + ".txt";
  infile.open(GT_file_path);
  // extract poses
  while (!infile.eof()) {
    std::getline(infile, line, '\n');
    if (line.size() == 0) { break; }
    std::vector<std::string> out;
    tokenize(line, delim, out);
    std::string image_file = out[0];
    std::string image_path = image_folder + "/" + image_file;
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    std::string cam_model = image_file.substr(10, 2);
    std::string cam_path;
    if (cam_model == "c0") {
      cam_path = cmu_path + "/c0.json";
    } else {
      cam_path = cmu_path + "/c1.json";
    }
    // get pose
    Eigen::Quaterniond q(atof(out[1].c_str()), atof(out[2].c_str()),
                         atof(out[3].c_str()), atof(out[4].c_str()));
    Eigen::Matrix3d R;
    beam::quat2rot(q, R);
    Eigen::Vector3d t;
    t << atof(out[5].c_str()), atof(out[6].c_str()), atof(out[7].c_str());
    Eigen::Matrix4d pose;
    pose.block<3, 3>(0, 0) = R;
    pose.block<3, 1>(0, 3) = t.transpose();
    Eigen::Vector4d v{0, 0, 0, 1};
    pose.row(3) = v;
    id->AddImage(image, pose, cam_path, image_file, false);
  }
  id->SaveDatabase();
  return id;
}

int main(int argc, char* argv[]) {
  std::shared_ptr<relocalization::ImageDatabase> id;
  cmu_path = std::string(argv[1]);
  slice_num = std::string(argv[2]);
  std::string vocab_path = "/home/jake/data/orbvoc.dbow3";

  LoadCameraModels();

  std::string slice_path = cmu_path + "/slice" + slice_num;
  std::string dbow_db_path = slice_path + "/bow_db.dbow3";
  if (boost::filesystem::exists(dbow_db_path)) {
    id = std::make_shared<relocalization::ImageDatabase>(
        slice_path + "/", slice_path + "/database");
  } else {
    id = BuildDB(slice_path, vocab_path);
  }
  matcher = relocalization::ImageToImage(id, det, desc, mat);
  EvaluateQueryImages();
}
