#include <ImageDatabase.h>
#include <beam_calibration/TfTree.h>
#include <beam_cv/descriptors/ORBDescriptor.h>
#include <beam_cv/detectors/ORBDetector.h>
#include <beam_mapping/Poses.h>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

ros::Time GetTimeStampFromJson(std::string path);

int main() {
  std::shared_ptr<beam_cv::ORBDetector> detector =
      std::make_shared<beam_cv::ORBDetector>();
  std::shared_ptr<beam_cv::ORBDescriptor> descriptor =
      std::make_shared<beam_cv::ORBDescriptor>();

  nlohmann::json J;
  std::string file_location = __FILE__;
  file_location.erase(file_location.end() - 20, file_location.end());
  file_location += "config/build_db_config.json";
  std::ifstream file(file_location);
  file >> J;
  // initialize db with descriptor, detector and vocabulary
  std::string vocab_loc = J["vocabulary_path"];
  std::shared_ptr<relocalization::ImageDatabase> database =
      std::make_shared<relocalization::ImageDatabase>(vocab_loc, descriptor,
                                                      detector, "/home/jake/data/imageDB/");
  // create tftree
  std::string tree_loc = J["extrinsics_path"];
  beam_calibration::TfTree tree;
  tree.LoadJSON(tree_loc);
  std::string to_frame = J["camera_frame"];
  std::string from_frame = "hvlp_link";
  Eigen::Matrix4d transform =
      tree.GetTransformEigen(to_frame, from_frame).matrix();
  // create poses object
  std::string poses_loc = J["poses_path"];
  beam_mapping::Poses poses;
  poses.LoadFromTXT(poses_loc);

  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>
      pose_matrices = poses.GetPoses();
  std::string intrinsics_path =
      "/home/jake/data/Market_Square/intrinsics/F1.json";
  // loop through image folder and add each to database
  std::vector<ros::Time> stamps = poses.GetTimeStamps();
  std::string images_folder = J["images_folder"];
  if (boost::filesystem::is_directory(images_folder)) {
    for (auto& entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(images_folder), {})) {
      if (boost::filesystem::is_directory(entry.path())) {
        // load image
        std::string image_path = entry.path().string() + "/BGRImage.jpg";
        cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
        // load info json and get timestamp
        std::string info_path = entry.path().string() + "/ImageBridgeInfo.json";
        ros::Time time_stamp = GetTimeStampFromJson(info_path);
        int idx;
        for (int i = 0; i < stamps.size(); i++) {
          ros::Time t = stamps[i];
          if (t == time_stamp) { idx = i; }
        }
        Eigen::Matrix4d T = pose_matrices[idx].matrix();
        Eigen::Matrix4d pose = T * transform;
        database->AddImage(image, pose, intrinsics_path);
      }
    }
  }
  database->SaveDatabase();
}

ros::Time GetTimeStampFromJson(std::string path) {
  nlohmann::json info_json;
  std::ifstream info_file(path);
  info_file >> info_json;
  ros::Time time_stamp;
  uint64_t time = info_json["time_stamp"];
  std::string time_s = std::to_string(time);
  uint64_t n_sec =
      std::stod(time_s.substr(time_s.length() - 9, time_s.length()));
  uint64_t sec = std::stod(time_s.substr(0, time_s.length() - 9));
  time_stamp.sec = sec;
  time_stamp.nsec = n_sec;
  return time_stamp;
}