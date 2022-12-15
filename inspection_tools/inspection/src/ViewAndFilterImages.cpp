#include <gflags/gflags.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/gflags.h>

#include <inspection/ImageDatabase.h>

DEFINE_string(camera_list, "",
              "Full path to cameras list json file (Required).");
DEFINE_validator(camera_list, &beam::gflags::ValidateJsonFileMustExist);
DEFINE_string(output_camera_list, "",
              "Full path to output cameras list json file (Required).");
DEFINE_validator(output_camera_list, &beam::gflags::ValidateCannotBeEmpty);
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

  ImageDatabase image_db_out(FLAGS_output_camera_list, image_type);

  if (image_db_orig.GetImagesFilename() == FLAGS_images_filename) {
    BEAM_ERROR("Input images filename value ({}) is the same as the original "
               "data, exiting.",
               FLAGS_images_filename);
    return 1;
  }

  image_db_out.SetImagesFilename(FLAGS_images_filename);

  for (CameraList::iterator iter = image_db_orig.CamerasBegin();
       iter != image_db_orig.CamerasEnd(); iter++) {
    const std::string& cam_name = iter->first;
    const ImageList& image_list = iter->second;
    std::string camera_dir = beam::CombinePaths(root_directory_, cam_name);
    for (const std::string& image_name : image_list) {
      cv::Mat image;
      if (image_type == ImageContainerType::IMAGE_BRIDGE) {
        std::string image_path = beam::CombinePaths(camera_dir, image_name);
        std::string image_json_path =
            beam::CombinePaths(image_path, "ImageInfo.json");
        beam_containers::ImageBridge img;
        img.LoadFromJSON(image_json_path);
        if (img.IsBGRImageSet()) {
          image = img.GetBGRImage().clone();
        } else if (img.IsIRImageSet()) {
          image = img.GetIRImage().clone();
        } else {
          BEAM_ERROR("No image in image container located at {}. Skipping",
                     image_json_path);
          continue;
        }
      } else if (image_type == ImageContainerType::NONE) {
        std::string image_path =
            beam::CombinePaths(camera_dir, image_name + ".jpg");
        image = cv::imread(image_path);
      } else {
        BEAM_CRITICAL("image container type not yet implemented. Options: "
                      "NONE, IMAGE_BRIDGE, input: {}. Exiting",
                      image_type);
        return 1;
      }

      // TODO visualize image, and add to new only if selected
    }
  }

  return 0;
}
