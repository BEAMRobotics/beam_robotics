import numpy as np
import json
import matplotlib.pyplot as plt
import argparse
import cv2
import json
import os
import pathlib
from segment_anything import SamPredictor, sam_model_registry


'''
Example Command:

python3 ~/catkin_ws/src/beam_robotics/pipelines/run_sam.py \
--ckpt ~/data/sam/sam_vit_h_4b8939.pth \
--images_directory /home/nick/d/results/image_extractor/ \
--camera_list F1_link --skip_first 10
'''


def parse_args():
    parser = argparse.ArgumentParser(
        description='Iterate through an image database extracted from the ImageExtractor binary and label images using Segment Anything')
    parser.add_argument(
        '--ckpt',
        dest='sam_checkpoint',
        required=True,
        type=str,
        help='path to segment anything checkpoint (.pth)',
    )
    parser.add_argument(
        '--images_directory',
        dest='images_directory',
        required=False,
        type=str,
        default=50,
        help='path to images directory, should contain cameras list json',
    )
    parser.add_argument(
        '--camera_list_file',
        dest='camera_list_file',
        required=False,
        type=str,
        default="CameraListNew",
        help='Name of the camera list file. Usually CameraList of CameraListNew',
    )
    parser.add_argument(
        '--skip_first',
        dest='skip_first',
        required=False,
        type=int,
        default=0,
        help='how many images to skip at start',
    )
    parser.add_argument(
        '--camera_list',
        dest='camera_list',
        required=False,
        default=[],
        type=str,
        nargs="+",
        help='camera names to use, if empty we will go through all. '
    )
    args = parser.parse_args()
    return args


class ImageExtractorIterator:
    def __init__(self, image_extractor_path: str, camera_list_file="CameraList", cameras=[], image_filename="BGRImage.jpg"):
        self.image_extractor_path = image_extractor_path
        self.camera_list_path = os.path.join(
            image_extractor_path, camera_list_file + ".json")
        self.cameras = cameras
        self.camera_id = 0
        self.image_id = 0
        self.image_filename = image_filename

        print(f"Reading camera list file: {self.camera_list_path}")
        with open(self.camera_list_path, 'r') as f:
            camera_list_data = json.load(f)
            self.images_filename = camera_list_data["ImagesFilename"]
            if not self.cameras:
                self.cameras = camera_list_data["Cameras"]
            else:
                for camera in self.cameras:
                    assert camera in camera_list_data["Cameras"], f"camera {camera} not found"

        self._get_current_images()

    def _get_current_images(self):
        self.current_images_list = []
        images_list_path = os.path.join(
            self.image_extractor_path, self.cameras[self.camera_id], f"{self.images_filename}.json")
        print(f"getting images list from {images_list_path}")
        with open(images_list_path, 'r') as f:
            images_data = json.load(f)
            for image in images_data["Images"]:
                image_container_path = os.path.join(
                    self.image_extractor_path, self.cameras[self.camera_id], f"{image}")
                assert os.path.exists(
                    image_container_path), f"image container does not exist: {image_path}"
                image_path = os.path.join(
                    image_container_path, self.image_filename)
                if os.path.exists(image_path):
                    self.current_images_list.append(image_path)

    def next(self):
        image_id = self.image_id
        if image_id > len(self.current_images_list) - 1:
            self.camera_id += 1
            self.image_id = 0
            image_id = 0
            if self.camera_id > len(self.cameras) - 1:
                return None
            self._get_current_images()
        else:
            self.image_id += 1
        return self.current_images_list[image_id]

    def skip_n(self, n: int):
        image_path = ""
        for i in range(n):
            image_path = self.next()
        return image_path


def get_clicked_points(image_path, downsize_factor=0.3):
    # Load the image using OpenCV
    image = cv2.imread(image_path)
    h = image.shape[0]
    w = image.shape[1]
    img = cv2.resize(
        image, (int(w * downsize_factor), int(h * downsize_factor)))

    # Define a function to handle mouse click event

    def onclick(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            nonlocal current_clicked_point  # Use nonlocal to modify the variable
            current_clicked_point = (x / downsize_factor, y / downsize_factor)

    # Initialize clicked_point variable
    current_clicked_point = None
    clicked_points = []
    labels = []

    # Create a window and display the image
    cv2.namedWindow("Select a point on the image")
    cv2.imshow("Select a point on the image", img)

    # Connect the function to the mouse click event
    cv2.setMouseCallback("Select a point on the image", onclick)

    # Wait for the user to click
    print("\nSelect a point on the image and then press:"
          "\n'y' to mark as positive, "
          "\n'n' to mark as negative, "
          "\n'd' to remove last, "
          "\nand 'enter' to complete adding points or skip image if no points selected, or"
          "\n'q' to quit")
    while True:
        k = cv2.waitKey(0)
        if k == 121:
            print("adding positive point prompt")
            center = (int(current_clicked_point[0] * downsize_factor),
                      int(current_clicked_point[1] * downsize_factor))
            cv2.circle(img, center, 5, (0, 255, 0), 2)
            cv2.imshow("Select a point on the image", img)
            clicked_points.append(current_clicked_point)
            labels.append(1)
        elif k == 110:
            print("adding negative point prompt")
            center = (int(current_clicked_point[0] * downsize_factor),
                      int(current_clicked_point[1] * downsize_factor))
            cv2.circle(img, center, 5, (0, 0, 255), 2)
            cv2.imshow("Select a point on the image", img)
            clicked_points.append(current_clicked_point)
            labels.append(0)
        elif k == 13:
            print("done selecting points")
            break
        elif k == 100:
            print("removing last point if available")
            if clicked_points:
                center = (int(clicked_points[-1][0] * downsize_factor),
                          int(clicked_points[-1][1] * downsize_factor))
                cv2.circle(img, center, 5, (0, 0, 0), 2)
                cv2.imshow("Select a point on the image", img)
                clicked_points = clicked_points[:-1]
                labels = labels[:-1]
        elif k == 113:
            print("quitting labeling!")
            cv2.destroyAllWindows()
            raise Exception("User selected to quit")
        else:
            print(f"invalid key, options: y, n, d, enter, q")

    cv2.destroyAllWindows()
    return clicked_points, labels


def show_mask(mask, ax, random_color=False):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30/255, 144/255, 255/255, 0.6])
    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    ax.imshow(mask_image)


def show_points(coords, labels, ax, marker_size=375):
    pos_points = coords[labels == 1]
    neg_points = coords[labels == 0]
    ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green',
               marker='*', s=marker_size, edgecolor='white', linewidth=1.25)
    ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red',
               marker='*', s=marker_size, edgecolor='white', linewidth=1.25)


def get_mask(predictor, image_path, image, image_bgr):
    while True:
        coords, labels = get_clicked_points(image_path)
        if not coords or not labels:
            print("no labels made, skipping image")
            return

        # convert to numpy
        num_points = len(coords)
        input_point = np.zeros((num_points, 2))
        input_label = np.array(labels)
        for i in range(num_points):
            input_point[i][0] = coords[i][0]
            input_point[i][1] = coords[i][1]

        if not coords:
            print("No coordinates selected, skipping to next image")
            return

        print("predicting masks")
        predictor.set_image(image)
        mask, score, logit = predictor.predict(
            point_coords=input_point,
            point_labels=input_label,
            multimask_output=False,
        )

        print("Showing image masks, close window to continue")
        plt.imshow(image)
        show_points(input_point, input_label, plt.gca())
        show_mask(mask, plt.gca())
        plt.axis('off')
        plt.show()

        user_input = input(
            "accept this mask? (y - yes, n - no, retry, s - skip to next without saving):").lower()
        if user_input == 'y':
            break
        elif user_input == 'n':
            continue
        elif user_input == 's':
            return
        else:
            print("Invalid input. Redoing mask generation.")
            continue

    label = 1
    while True:
        user_input = input(
            "what kind of label is this? "
            "\n1 - crack, "
            "\n2 - delamination, "
            "\n3 - corrosion, "
            "\n4 - spall\n").lower()
        if user_input == '1':
            label = 1
            break
        elif user_input == '2':
            label = 2
            break
        elif user_input == '3':
            label = 3
            break
        elif user_input == '4':
            label = 4
            break
        else:
            print("Invalid input. Options: 1-4.")
            continue

    # Create 3 images:
    img_h = mask.shape[1]
    img_w = mask.shape[2]
    bgr_mask = np.zeros((img_h, img_w))
    bgr_mask_viz = np.zeros((img_h, img_w))
    bgr_mask_overlay = image_bgr.copy()
    save_path = pathlib.Path(image_path).parent
    for j in range(mask.shape[1]):
        for k in range(mask.shape[2]):
            if mask[0][j][k]:
                bgr_mask_overlay[j][k] = (0, 0, 255)
                bgr_mask_viz[j][k] = 255/2
                bgr_mask[j][k] = int(label)

    bgr_mask_save_path = os.path.join(save_path, "BGRMask.png")
    bgr_mask_viz_save_path = os.path.join(save_path, "BGRMaskViz.png")
    bgr_mask_overlay_save_path = os.path.join(save_path, "BGRMaskOverlay.jpg")
    print("saving: ", bgr_mask_save_path)
    cv2.imwrite(bgr_mask_save_path, bgr_mask)
    cv2.imwrite(bgr_mask_viz_save_path, bgr_mask_viz)
    cv2.imwrite(bgr_mask_overlay_save_path, bgr_mask_overlay)

    # update json
    json_path = os.path.join(save_path, "ImageInfo.json")
    print(f"updating image info: {json_path}")
    image_info = {}
    with open(json_path, "r") as f:
        image_info = json.load(f)
    image_info["bgr_mask_method"] = "SAM"
    image_info["is_bgr_mask_set"] = True

    print("image_info\n", image_info)
    with open(json_path, 'w') as f:
        json.dump(image_info, f, indent=4)


def main():
    args = parse_args()
    sam_model = sam_model_registry["default"](checkpoint=args.sam_checkpoint)
    predictor = SamPredictor(sam_model)

    image_iter = ImageExtractorIterator(
        image_extractor_path=args.images_directory,
        camera_list_file=args.camera_list_file,
        cameras=args.camera_list
    )
    if args.skip_first > 0:
        image_iter.skip_n(args.skip_first)

    while True:
        image_path = image_iter.next()
        if image_path is None:
            break

        image_bgr = cv2.imread(image_path)
        image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        get_mask(predictor, image_path, image, image_bgr)


if __name__ == "__main__":
    main()
