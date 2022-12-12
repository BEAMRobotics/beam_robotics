import argparse
import sys
import json
import os
import logging
import cv2
import shutil

camera_list_ = ""
new_camera_list_filename_ = ""
new_image_list_filename_ = ""
view_scale_ = 50


def parse_args():
    global camera_list_
    global new_camera_list_filename_
    global new_image_list_filename_
    global view_scale_
    parser = argparse.ArgumentParser(
        description=
        'View images from an image container that have been extracted from '
        'inspection_extract_images binary. This also allows you to select which '
        'images to keep and which to discard. Note that discarding images does '
        'not delete the data, it only removes the image from the images list json.')
    parser.add_argument(
        '--data',
        dest='camera_list',
        required=True,
        type=str,
        help=
        'path to data json listing cameras and image lists',
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
        '--new_camera_list_filename',
        dest='new_camera_list_filename',
        required=False,
        default="CamerasListNew",
        type=str,
        help=
        'set the name for the new cameras list filename. '
        'This file will also have the update names for each images list file.',
    )
    parser.add_argument(
        '--new_image_list_filename',
        dest='new_image_list_filename',
        required=False,
        default="ImageListNew",
        type=str,
        help=
        'set the name for the new image list filename. '
        'E.g., if saving images for crack detection: CrackImages',
    )
    args = parser.parse_args()

    logging.info("read data path: %s", args.camera_list)
    camera_list_ = args.camera_list
    new_camera_list_filename_ = args.new_camera_list_filename
    new_image_list_filename_ = args.new_image_list_filename
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


def show_camera_images(camera_data_dir, images_filename):
    global new_image_list_filename_
    image_list_path = os.path.join(camera_data_dir, images_filename)

    if not os.path.exists(image_list_path):
        logging.error("images list json file does not exist at: %s",
                      image_list_path)
        return

    logging.info("reading image list from: %s", image_list_path)
    image_list_file = open(image_list_path)
    image_list_json_data = json.load(image_list_file)
    image_list = image_list_json_data["Images"]
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
    output_image_list["Images"] = image_list_keep
    images_filename_new = os.path.join(camera_data_dir, new_image_list_filename_)
    logging.info("writing new images list to: %s", images_filename_new)
    with open(images_filename_new, 'w') as f:
        json.dump(output_image_list, f)


def read_cameras_list():
    global camera_list_
    global new_camera_list_filename_
    global new_image_list_filename_

    if not os.path.exists(camera_list_):
        logging.error("camera list json file does not exist at: %s",
                      camera_list_)
        return

    logging.info("opening camera list json file: %s", camera_list_)
    file = open(camera_list_)
    json_data = json.load(file)
    camera_list = json_data["Cameras"]
    file.close()
    logging.info("read the following cameras: %s", str(json_data["Cameras"]))
    cameras_dir = os.path.dirname(camera_list_)
    for camera_name in json_data["Cameras"]:
        camera_data_dir = os.path.join(cameras_dir, camera_name)
        show_camera_images(camera_data_dir, json_data["ImagesFilename"])

    # save new cameras list
    json_data["ImagesFilename"] = new_image_list_filename_
    camera_list_path = os.path.join(cameras_dir,  new_camera_list_filename_)
    with open(camera_list_path, 'w') as f:
        json.dump(json_data, f)


def main(args):
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
    logging.info("running view_and_filter_images tool")
    parse_args()
    read_cameras_list()


if __name__ == "__main__":
    main(sys.argv[1:])