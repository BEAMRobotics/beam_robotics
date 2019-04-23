#define PCL_NO_PRECOMPILE

#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/kdtree.h>

#include "inspection/BagAnalyzer.h"
#include "inspection/LadybugLabeler.h"
#include "inspection/MapLabeler.h"
#include <beam_containers/Utilities.h>
#include <beam_utils/time.hpp>

namespace inspection {

LadybugLabeler::LadybugLabeler(std::string config_file_location)
    : json_labeler_filepath_(config_file_location) {
  // Process core json file
  //  ProcessJSONConfig();

  // Fill tf tree object with robot poses & extrinsics
  FillTFTree();

  for (int i = 2; i < 13; i++) {
    std::string fname = {};
    if (i < 11) {
      fname = "/home/steve/ladybug_market/ImageBridge" + std::to_string(i) +
              "/scan_00" + std::to_string(i - 1) + ".pcd";
    } else {
      fname = "/home/steve/ladybug_market/ImageBridge" + std::to_string(i) +
              "/scan_0" + std::to_string(i - 1) + ".pcd";
    }

    DefectCloud::Ptr cloud = boost::make_shared<DefectCloud>();

    if (pcl::io::loadPCDFile<BridgePoint>(fname, *cloud) == -1) {
      PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    }
    //    if (i == 2) { defect_pointcloud_ = cloud; }
    scan_clouds.push_back(cloud);
    std::cout << "Loaded scan of size: " << cloud->points.size() << std::endl;
  }

  std::vector<int> cam_ids = {0, 2, 3, 4, 5};
  for (int cam : cam_ids) {
    Camera camera{cam, "/home/steve/ladybug.conf"};
    cameras_.push_back(std::move(camera));
  }

  defect_clouds_.resize(5);

  for (int i = 2; i < 12; i++) {
    for (int cam = 0; cam < 5; cam++) {
      std::cout << "Camera id: " << cam << std::endl;

      //      beam_containers::ImageBridge img_bridge_;
      std::string bgr_file = "/home/steve/ladybug_market/ImageBridge" +
                             std::to_string(i) + "/bgr/cam_0" +
                             std::to_string(cam_ids[cam]) + ".png";
      std::string bgr_mask_file = "/home/steve/ladybug_market/ImageBridge" +
                                  std::to_string(i) + "/mask/cam_0" +
                                  std::to_string(cam_ids[cam]) + ".png";
      std::cout << "BGR file: " << bgr_file << std::endl;
      std::cout << "BGR MASK file : " << bgr_mask_file << std::endl;
      cv::Mat bgr_img = cv::imread(bgr_file);
      cv::Mat bgr_mask = cv::imread(bgr_mask_file, cv::IMREAD_GRAYSCALE);
      img_bridge_.SetBGRMask(bgr_mask);
      img_bridge_.SetBGRImage(bgr_img);
      int seq = i - 1;
      img_bridge_.SetImageSeq(seq);
      std::cout << "ImageBridge" << std::to_string(i) << " : Scan " << seq
                << std::endl;

      std::cout << bgr_img.rows << std::endl;
      std::cout << bgr_img.cols << std::endl;
      std::cout << bgr_mask.rows << std::endl;
      std::cout << bgr_mask.cols << std::endl;

      Camera* camera = &(cameras_[cam]);
      DefectCloud::Ptr colored_cloud = ProjectImgToMap(img_bridge_, camera);

      /*      std::cout << "Writing pcd" << std::endl;
            pcl::io::savePCDFileBinary("/home/steve/test_cloud.pcd",
         *colored_cloud);

            std::cout << colored_cloud->points.size() << std::endl;
            defect_clouds_[cam].push_back(colored_cloud);

            PointCloudXYZRGB::Ptr cloud_rgb =
         boost::make_shared<PointCloudXYZRGB>();
            pcl::copyPointCloud(*colored_cloud, *cloud_rgb);
            rgb_clouds.push_back(cloud_rgb);*/

      pcl::io::savePCDFileBinary("/home/steve/cloud_image" + std::to_string(i) +
                                     "_cam " + std::to_string(cam_ids[cam]) +
                                     ".pcd",
                                 *colored_cloud);
    }
  }
}

void LadybugLabeler::DrawFinalMap() {
  int id = 0;
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
      rgb_field;

  PointCloudXYZRGB::Ptr rgb_pc = boost::make_shared<PointCloudXYZRGB>();
  pcl::copyPointCloud(*cloud_combiner_.GetCombinedCloud(), *rgb_pc);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      rgb_pc);

  viewer->addPointCloud<pcl::PointXYZRGB>(rgb_pc, rgb, "Final");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Final");

  viewer->setBackgroundColor(1, 1, 1);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
}

void LadybugLabeler::ProcessJSONConfig() {
  // Start by loading the root json file & assigning variables
  std::ifstream json_config_stream(json_labeler_filepath_);
  json_config_stream >> json_config_;
  images_file_name_ = json_config_["params"]["images_path"];
  extrinsics_file_name_ = json_config_["params"]["extrinsics_path"];
  path_to_camera_calib_ = json_config_["params"]["intrinsics_path"];

  // Next we load in the CamerasList.json
  nlohmann::json json_cameras_list;
  std::ifstream camera_list_stream(images_file_name_ + "/CamerasList.json");
  camera_list_stream >> json_cameras_list;
  std::cout << json_cameras_list << std::endl;

  // Now we create Camera objects for each camera defined in the
  // CamerasList.json file.
  for (const auto& camera_name : json_cameras_list["Items"]) {
    std::string camera_folder_path =
        images_file_name_ + "/" + std::string(camera_name);
    std::cout << camera_folder_path << std::endl;
  }
}

void LadybugLabeler::FillTFTree() {
  std::string bag_name = "/home/steve/roben_scan_2019-04-11-20-09-55.bag";
  //  inspection::BagAnalyzer bag_file_ = {bag_name};
  rosbag::Bag bag_file_{bag_name};
  std::vector<std::string> topics;
  //  topics.emplace_back("/tf");
  topics.emplace_back("/tf_static");

  rosbag::View tf_view = {bag_file_, rosbag::TopicQuery("/tf")};
  ros::Duration t = {tf_view.getEndTime() - tf_view.getBeginTime()};

  // Store "tf" transforms in buffer
  /*  for (const auto& msg_instance : tf_view) {
      auto tf_message = msg_instance.instantiate<tf2_msgs::TFMessage>();
      if (tf_message != nullptr) {
        for (const auto& tf : tf_message->transforms) {
          if (!tf_tree.AddTransform(tf))
            std::cout << "Adding message failed" << std::endl;
        }
      }
    }*/

  rosbag::View tf_static_view = {bag_file_, rosbag::TopicQuery("/tf_static")};
  // Store "tf_static" transforms in buffer
  for (const auto& msg_instance : tf_static_view) {
    auto tf_message = msg_instance.instantiate<tf2_msgs::TFMessage>();
    if (tf_message != nullptr) {
      for (const auto& tf : tf_message->transforms) {
        tf_tree.AddTransform(tf, true);
      }
    }
  }
}

void LadybugLabeler::SaveLabeledClouds() {
  std::string root_cloud_folder = images_file_name_ + "/../clouds";
  for (size_t cam = 0; cam < defect_clouds_.size(); cam++) {
    int cloud_number = 1;
    for (const auto& cloud : defect_clouds_[cam]) {
      std::string file_name =
          cameras_[cam].camera_id + "_" + std::to_string(cloud_number) + ".pcd";
      pcl::io::savePCDFileBinary(root_cloud_folder + "/" + file_name, *cloud);
      cloud_number++;
    }
    std::cout << "Saved " << cloud_number - 1
              << " labeled point clouds from Camera: "
              << cameras_[cam].camera_id << std::endl;
  }

  pcl::io::savePCDFileBinary(root_cloud_folder + "/final.pcd",
                             *cloud_combiner_.GetCombinedCloud());
}

void LadybugLabeler::DrawColoredClouds() {
  int id = 0;
  std::vector<
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>>
      rgb_fields;
  for (auto& cloud : rgb_clouds) {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
        cloud);
    rgb_fields.push_back(rgb);
    std::cout << "Adding point cloud with id: " << id << std::endl;
    std::cout << "  Point cloud size = " << cloud->points.size() << std::endl;
    viewer->addPointCloud<pcl::PointXYZRGB>(rgb_clouds[id], rgb_fields[id],
                                            std::to_string(id));
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, std::to_string(id));
    id++;
  }

  viewer->setBackgroundColor(1, 1, 1);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
}

DefectCloud::Ptr LadybugLabeler::TransformMapToImageFrame(ros::Time tf_time,
                                                          std::string frame_id,
                                                          int scan_number) {
  std::string to_frame = frame_id;
  std::string from_frame = "m3d_link";

  geometry_msgs::TransformStamped transform_msg =
      tf_tree.GetTransform(to_frame, from_frame, tf_time);

  std::cout << "Transform lookup time = " << tf_time << std::endl;
  std::cout << "Transform = " << transform_msg << std::endl;

  auto transformed_cloud = boost::make_shared<DefectCloud>();

  tf::Transform tf_;
  tf::transformMsgToTF(transform_msg.transform, tf_);
  pcl_ros::transformPointCloud(*scan_clouds[scan_number - 1],
                               *transformed_cloud, tf_);

  tf_temp_ = tf_.inverse();
  std::cout
      << "Successfully transformed point cloud from map frame to image frame"
      << frame_id << "." << std::endl;

  return transformed_cloud;
}

void LadybugLabeler::PlotFrames(std::string frame_id, PCLViewer viewer) {
  std::vector<Eigen::Affine3f> coord_frames;

  for (const auto& pose : final_poses_) {
    // beam::chronoToRosTime(pose.first);
    ros::Time time = TimePointToRosTime(pose.first); //
    std::string to_frame = "map";
    geometry_msgs::TransformStamped g_tf_stamped =
        tf_tree.GetTransform(to_frame, frame_id, time);

    Eigen::Isometry3d eig = tf2::transformToEigen(g_tf_stamped);
    Eigen::Affine3f affine_tf(eig.cast<float>());

    std::stringstream unique_id;
    unique_id << frame_id << "_" << time.sec;

    /*    std::cout << "Adding affine_tf : " << unique_id.str()
                  << " with transform: " << g_tf_stamped.transform.translation.x
                  << std::endl;*/

    viewer->addCoordinateSystem(0.5, affine_tf, unique_id.str());
  }
}

DefectCloud::Ptr
    LadybugLabeler::ProjectImgToMap(beam_containers::ImageBridge img_container,
                                    Camera* camera) {
  TimePoint img_time = img_bridge_.GetTimePoint();
  ros::Time ros_img_time{0};       // = TimePointToRosTime(img_time);
  beam::Vec2 img_dims{2048, 2464}; // = camera->cam_intrinsics->GetImgDims();

  std::string camera_frame_id =
      "ladybug_cam" + std::to_string(camera->camera_id);
  // Get map in camera frame
  DefectCloud::Ptr map_cloud = TransformMapToImageFrame(
      ros_img_time, camera_frame_id,
      img_bridge_.GetImageSeq()); // camera->cam_intrinsics->GetFrameId());

  pcl::io::savePCDFileBinary("/home/steve/test_cloud2.pcd", *map_cloud);

  auto xyz_cloud = boost::make_shared<PointCloudXYZ>();
  pcl::copyPointCloud(*map_cloud, *xyz_cloud);
  camera->colorizer->SetPointCloud(xyz_cloud);
  DefectCloud::Ptr return_cloud = boost::make_shared<DefectCloud>();

  if (img_container.IsBGRImageSet()) {
    cv::Mat bgr_img = img_container.GetBGRImage();
    camera->colorizer->SetImage(bgr_img);

    // Get colored cloud & remove uncolored points

    auto xyzrgb_cloud = camera->colorizer->ColorizePointCloud();
    //    xyzrgb_cloud = camera->colorizer->ColorizePointCloud();
    xyzrgb_cloud->points.erase(
        std::remove_if(xyzrgb_cloud->points.begin(), xyzrgb_cloud->points.end(),
                       [](auto& point) {
                         return (point.r == 0 && point.g == 0 && point.b == 0);
                       }),
        xyzrgb_cloud->points.end());

    xyzrgb_cloud->width = xyzrgb_cloud->points.size();
    std::cout << "XYZRGB cloud size = " << xyzrgb_cloud->width << std::endl;
    std::cout << "XYZRGB points size = " << xyzrgb_cloud->points.size()
              << std::endl;

    pcl::copyPointCloud(*xyzrgb_cloud, *return_cloud);
  }
  if (img_container.IsBGRMaskSet()) {
    cv::Mat bgr_mask = img_container.GetBGRMask();
    camera->colorizer->SetImage(bgr_mask);
    auto masked_cloud = camera->colorizer->ColorizeMask();

    pcl::search::KdTree<BridgePoint> kdtree;
    kdtree.setInputCloud(return_cloud);
    for (const auto& search_point : *masked_cloud) {
      if (search_point.crack == 0 && search_point.delam == 0 &&
          search_point.corrosion == 0) {
        continue;
      }
      int K = 1;
      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);
      if (kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0) {
        return_cloud->points[pointIdxNKNSearch[0]].crack += search_point.crack;
        return_cloud->points[pointIdxNKNSearch[0]].delam += search_point.delam;
        return_cloud->points[pointIdxNKNSearch[0]].corrosion +=
            search_point.corrosion;
        // 1 crack 2 spall 3 patches
      }
    }
  }

  pcl_ros::transformPointCloud(*return_cloud, *return_cloud, tf_temp_);

  return return_cloud;
}

} // end namespace inspection
