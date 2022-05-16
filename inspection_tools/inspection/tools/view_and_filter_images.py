import argparse
import sys
import json
import os
import logging
import cv2
import shutil

data_path_ = ""
save_original_ = True
view_scale_ = 50


def parse_args():
    global data_path_
    global save_original_
    global view_scale_
    parser = argparse.ArgumentParser(
        description=
        'View images from an image container that have been extracted from '
        'inspection_extract_images binary. This also allows you to select which '
        'images to keep and which to discard. Note that discarding images does '
        'not delete the data, it only removes the image from the ImagesList.')
    parser.add_argument(
        '--data',
        dest='data_path',
        required=True,
        type=str,
        help=
        'path to data. This should contain a CameraList.json file which points '
        'to all the data',
    )
    parser.add_argument(
        '--view_scale',
        dest='view_scale',
        required=False,
        type=float,
        default=50,
        help='rescale image for viewing by percentage (0-100)',
    )
    parser.add_argument(
        '--save_original',
        dest='save_original',
        required=False,
        default=True,
        type=bool,
        help=
        'if set to true, this will copy the original ImagesList.json for each '
        'camera and save it as ImageListOriginal.json',
    )
    args = parser.parse_args()

    logging.info("read data path: %s", args.data_path)
    data_path_ = args.data_path
    save_original_ = args.save_original
    view_scale_ = args.view_scale


def display_image(image_path):
    global view_scale_

    if not os.path.exists(image_path):
        logging.error("image file does not exist at: %s", image_path)
        return

    image = cv2.imread(image_path)
    
    # resize image
    width = int(image.shape[1] * view_scale_ / 100)
    height = int(image.shape[0] * view_scale_ / 100)
    dim = (width, height)
    resized_image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)

    cv2.imshow('image', resized_image)
    keep_image = True
    if cv2.waitKey(0) == ord('n'):
        logging.info("removing image: %s", image_path)
        keep_image = False

    cv2.destroyAllWindows()
    return keep_image


def show_camera_images(camera_data_dir):
    global save_original_

    image_list_path = os.path.join(camera_data_dir, "ImagesList.json")

    if not os.path.exists(image_list_path):
        logging.error("ImagesList json file does not exist at: %s",
                      image_list_path)
        return

    # Save original only if it doesn't already exist
    if save_original_:
        image_list_path_original = os.path.join(camera_data_dir,
                                                "ImagesListOriginal.json")
        if os.path.exists(image_list_path_original):
            logging.info("original image list file exists, not replacing it.")
        else:
            logging.info("copying image list to: %s", image_list_path_original)
            shutil.copyfile(image_list_path, image_list_path_original)

    logging.info("reading image list from: %s", image_list_path)
    image_list_file = open(image_list_path)
    image_list_json_data = json.load(image_list_file)
    image_list = image_list_json_data["Items"]
    image_list_keep = []
    logging.info("read %d images", len(image_list))
    logging.info("displaying images, press 'n' to discard, or any key to keep")
    for image_name in image_list:
        image_container_path = os.path.join(camera_data_dir, image_name)
        image_info_json = os.path.join(image_container_path, "ImageInfo.json")
        image_info_file = open(image_info_json)
        image_info_file_json_data = json.load(image_info_file)
        image_path = ""
        if image_info_file_json_data["is_bgr_image_set"]:
            image_path = os.path.join(image_container_path, "BGRImage.jpg")
        elif image_info_file_json_data["is_ir_image_set"]:
            image_path = os.path.join(image_container_path, "IRImage.jpg")
        elif image_info_file_json_data["is_bgr_mask_set"]:
            image_path = os.path.join(image_container_path, "BGRMask.jpg")
        elif image_info_file_json_data["is_ir_mask_set"]:
            image_path = os.path.join(image_container_path, "IRMask.jpg")
        else:
            logging.error("image container empty for: %s",
                          image_container_path)
            continue

        save_image = display_image(image_path)

        if save_image:
            image_list_keep.append(image_name)

    image_list_file.close()
    output_image_list = {}
    output_image_list["Items"] = image_list_keep
    logging.info("writing new ImagesList to: %s", image_list_file)
    with open(image_list_path, 'w') as f:
        json.dump(output_image_list, f)


def read_metadata():
    global data_path_
    metadata_json = os.path.join(data_path_, "CamerasList.json")

    if not os.path.exists(metadata_json):
        logging.error("CameraList json file does not exist at: %s",
                      metadata_json)
        return

    logging.info("opening CameraList json file: %s", metadata_json)
    file = open(metadata_json)
    json_data = json.load(file)
    camera_list = json_data["Items"]
    file.close()
    logging.info("read the following cameras: %s", str(camera_list))
    for camera_name in camera_list:
        camera_data_dir = os.path.join(data_path_, camera_name)
        show_camera_images(camera_data_dir)


def main(args):
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
    logging.info("running view_and_filter_images tool")
    parse_args()
    read_metadata()


if __name__ == "__main__":
    main(sys.argv[1:])