#include <gflags/gflags.h>

#include <beam_containers/ImageBridge.h>
#include <beam_utils/filesystem.h>
#include <beam_utils/gflags.h>

#include <inspection/ImageDatabase.h>

DEFINE_string(camera_list, "",
              "Full path to cameras list json file (Required).");
DEFINE_validator(camera_list, &beam::gflags::ValidateJsonFileMustExist);
DEFINE_string(
    output_name, "CameraListNew",
    "Name of output camera list with filteres images. This will get saved at "
    "the same location as the input camera list. Default: CameraListNew");
DEFINE_string(
    images_filename, "",
    "File name for images list, this must be different than the original, "
    "otherwise the original data would be overridden (we check for this).");
DEFINE_validator(images_filename, &beam::gflags::ValidateCannotBeEmpty);
DEFINE_string(image_container_type, "IMAGE_BRIDGE",
              "Type of image container, options: NONE (i.e., jpg) or "
              "IMAGE_BRIDGE (see beam_containers).");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  using namespace inspection;

  ImageContainerType image_type;
  if (FLAGS_image_container_type == "NONE") {
    image_type = ImageContainerType::NONE;
  } else if (FLAGS_image_container_type == "IMAGE_BRIDGE") {
    image_type = ImageContainerType::IMAGE_BRIDGE;
  } else {
    BEAM_CRITICAL("invalid image_container_type input. Options: NONE, "
                  "IMAGE_BRIDGE, input: {}. Exiting",
                  FLAGS_image_container_type);
    return 1;
  }
  ImageDatabase image_db_orig(FLAGS_camera_list, image_type);
  image_db_orig.LoadMetadata();

  std::string output_camera_list = beam::CombinePaths(
      image_db_orig.GetCameraListRootPath(), FLAGS_output_name + ".json");
  ImageDatabase image_db_out(output_camera_list, image_type);

  if (image_db_orig.GetImagesFilename() == FLAGS_images_filename) {
    BEAM_ERROR("Input images filename value ({}) is the same as the original "
               "data, exiting.",
               FLAGS_images_filename);
    return 1;
  }

  image_db_out.SetImagesFilename(FLAGS_images_filename);

  BEAM_INFO("Displaying images. Press y to keep or n to discard");
  for (CameraList::iterator iter = image_db_orig.CamerasBegin();
       iter != image_db_orig.CamerasEnd(); iter++) {
    const std::string& cam_name = iter->first;
    const ImageList& image_list = iter->second;

    BEAM_INFO("Displaying images for camera: {}", cam_name);
    for (const std::string& image_name : image_list) {
      std::cout << "\n\nImage name: " << image_name << "\n";
      std::string image_path = image_db_orig.GetImagePath(cam_name, image_name);
      std::cout << "\n\nImage path: " << image_path << "\n";
      cv::Mat image;
      if (image_type == ImageContainerType::IMAGE_BRIDGE) {
        beam_containers::ImageBridge img;
        img.LoadFromJSON(image_path);
        if (img.IsBGRImageSet()) {
          image = img.GetBGRImage().clone();
        } else if (img.IsIRImageSet()) {
          image = img.GetIRImage().clone();
        } else {
          BEAM_ERROR("No image in image container located at {}. Skipping",
                     image_path);
          continue;
        }
      } else if (image_type == ImageContainerType::NONE) {
        image = cv::imread(image_path);
      } else {
        BEAM_CRITICAL("image container type not yet implemented. Options: "
                      "NONE, IMAGE_BRIDGE");
        throw std::runtime_error{"image container type not yet implemented"};
      }

      cv::imshow(image_name, image);
      bool valid_key{false};
      while (!valid_key) {
        int key = cv::waitKey();
        if (key == 121) {
          BEAM_INFO("Keeping image: {}", image_name);
          valid_key = true;
          image_db_out.AddImageMetadata(cam_name, image_name);
        } else if (key == 110) {
          BEAM_INFO("Rejecting image: {}", image_name);
          valid_key = true;
        } else {
          BEAM_INFO("Invalid key, enter y/n to keep/reject image");
        }
      }
    }
    image_db_out.WriteMetadata();
  }

  return 0;
}
