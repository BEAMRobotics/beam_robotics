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
    global images_list_
    global view_scale_

    parser = argparse.ArgumentParser(
        description=
        'View images from an image container that have been extracted from '
        'inspection_extract_images binary. This only views one set of image lists, i.e., one camera')
    parser.add_argument(
        '--images_list',
        dest='images_list',
        required=True,
        type=str,
        help='path to ImagesList.json',
    )
    parser.add_argument(
        '--view_scale',
        dest='view_scale',
        required=False,
        type=float,
        default=50,
        help='rescale image for viewing by percentage (0-100)',
    )
    args = parser.parse_args()

    images_list_ = args.images_list
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
    key_pressed = cv2.waitKey(0)
    if key_pressed == ord('n'):
        cv2.destroyAllWindows()
        return False
    elif key_pressed == ord('e'):
        logging.info("exit key selected")
        cv2.destroyAllWindows()
        return True    


def show_images():
    global images_list_

    if not os.path.exists(images_list_):
        logging.error("ImagesList json file does not exist at: %s",
                      images_list_)
        return

    logging.info("reading image list from: %s", images_list_)
    image_list_file = open(images_list_)
    image_list_json_data = json.load(image_list_file)
    image_list = image_list_json_data["Items"]
    logging.info("read %d images", len(image_list))
    logging.info("displaying images, press 'n' to skip to next, or 'e' to exit")

    root_dir = os.path.dirname(images_list_)
    for image_name in image_list:
        image_container_path = os.path.join(root_dir, image_name)
        image_info_json = os.path.join(image_container_path, "ImageInfo.json")
        image_info_file = open(image_info_json)
        image_info_file_json_data = json.load(image_info_file)
        image_path = ""
        if image_info_file_json_data["is_bgr_image_set"]:
            image_path = os.path.join(image_container_path, "BGRImage.jpg")
        elif image_info_file_json_data["is_ir_image_set"]:
            image_path = os.path.join(image_container_path, "IRImage.jpg")
        else:
            logging.error("image container empty for: %s",
                          image_container_path)
            continue

        exit_viewer = display_image(image_path)
        if exit_viewer:
            image_list_file.close()
            return

    image_list_file.close()


def main(args):
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
    logging.info("running view_and_filter_images tool")
    parse_args()
    show_images()


if __name__ == "__main__":
    main(sys.argv[1:])