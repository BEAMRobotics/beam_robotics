#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>
#include <ros/time.h>

namespace inspection {

/**
 * @brief list of paths to image containers (either folder for ImageBridge, or
 * jpg file for no image container)
 */
using ImageList = std::vector<std::string>;

/**
 * @brief map from camera name to list of images
 */
using CameraList = std::unordered_map<std::string, ImageList>;

enum class ImageContainerType { IMAGE_BRIDGE = 0, NONE };

/**
 * @brief the goal of this class is to contain all images associated with an
 * inspection dataset. We store a set of cameras, where each camera has a set of
 * images, and each image is a path to an image container from beam_containers,
 * or just a regular jpg image. The code throughout this class is based on the
 * assumption files are in the following format:
 *
 * CameraList.json
 * CameraName1/
 *    ImageList.json
 *    ImageName1
 *    ImageName1
 *    ...
 * CameraName2/
 *    ...
 * ...
 *
 * Where the jsons have the following contents:
 *
 * CameraList.json:
 *    {
 *      "Cameras":[
 *        "CameraName1",
 *        "CameraName2"
 *        ...
 *      ],
 *      "ImagesFilename": "ImagesList // this is used to find
 *                                       the image list json
 *    }
 *
 * ImageList.json:
 *    {
 *      "Images":[
 *        ImageName1,
 *        ImageName2,
 *        ...
 *      ]
 *    }
 *
 */
class ImageDatabase {
public:
  /**
   * @brief Construct a new Image Database object
   * @param camera_list_path required path to cameras list json file. If
   * loading from existing database, this must exist, and you must then call
   * LoadMetadata before using any data. Otherwise, this will get created when
   * calling WriteMetadata()
   * @param image_container_type way to store images, see struct def above. If
   * IMAGE_BRIDGE, each image will be stored as a separate folder to contain all
   * image data. If NONE, then each image will be saved as a jpg, all images
   * together in the same folder
   */
  ImageDatabase(const std::string& camera_list_path,
                const ImageContainerType& image_container_type =
                    ImageContainerType::IMAGE_BRIDGE);

  CameraList::iterator CamerasBegin() const;

  CameraList::iterator CamerasEnd() const;

  void SetImagesFilename(const std::string& images_filename);

  std::string GetImagesFilename() const;

  std::string GetCameraListPath() const;

  void LoadMetadata();

  /**
   * @brief write all metadata to disk. This creates a json file for each
   * camera, listing each image name, and creates a json file describing all
   * cameras
   */
  void WriteMetadata() const;

  /**
   * @brief add an image to the image container. This will immediately save the
   * image to disk within a folder named using the camera name. This will also
   * save the name of the image to later go into the metadata files
   */
  void AddImage(const std::string& camera_name, const cv::Mat& image,
                const ros::Time& time = ros::Time(0), bool is_distorted = true,
                const std::string& frame_id = "", bool is_ir_image = false,
                const std::string& dataset_name = "");

  ImageList ReadImageList(const std::string& image_list_path) const;

private:
  CameraList camera_list_;
  std::string camera_list_path_;
  std::string root_directory_;
  std::string images_filename_{"ImageList"};
  ImageContainerType image_container_type_{ImageContainerType::IMAGE_BRIDGE};
};

} // namespace inspection
