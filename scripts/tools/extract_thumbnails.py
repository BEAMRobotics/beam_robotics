import rosbag
import cv2
from cv_bridge import CvBridge
from PIL import Image
import os
import argparse
import json
from tqdm import tqdm
import tempfile
import shutil
import time


def get_bag_metadata(bag_file):
    """Retrieve ROS bag metadata including size and duration."""
    bag_size = os.path.getsize(bag_file) * 1e-9  # Convert bytes to MB

    with rosbag.Bag(bag_file, "r") as bag:
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        duration = end_time - start_time

    return {"bag_size_GB": round(bag_size, 2), "duration_seconds": round(duration, 2), "duration_minutes": round(duration / 60, 2)}


def extract_images(bag_file, image_topic, output_dir, thumbnail_width, interval):
    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Initialize ROS bag and OpenCV bridge
    bridge = CvBridge()
    with rosbag.Bag(bag_file, "r") as bag:
        last_time = None
        image_list = []
        count = 0
        for topic, msg, t in bag.read_messages(topics=[image_topic]):
            if last_time is None or (t.to_sec() - last_time >= interval):
                last_time = t.to_sec()

                # Convert ROS image message to OpenCV format
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

                # Convert OpenCV image to PIL image
                pil_image = Image.fromarray(
                    cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

                # Determine new height to maintain aspect ratio
                original_width, original_height = pil_image.size
                aspect_ratio = original_height / original_width
                thumbnail_height = int(thumbnail_width * aspect_ratio)

                # Create a thumbnail
                pil_image.thumbnail((thumbnail_width, thumbnail_height))

                # Save the thumbnail
                filename = f"thumbnail_{count}.jpg"
                filepath = os.path.join(
                    output_dir, filename)
                count += 1
                pil_image.save(filepath, "JPEG")
                image_list.append(filename)

                print(
                    f"Saved thumbnail: {filename} ({thumbnail_width}x{thumbnail_height})")

    metadata = get_bag_metadata(bag_file)
    metadata["thumbnails"] = image_list
    # Save metadata to JSON
    json_filename = os.path.join(output_dir, "metadata.json")
    with open(json_filename, "w") as json_file:
        json.dump(metadata, json_file, indent=4)

    print(f"Metadata saved: {json_filename}")

    print("Processing complete.")


def main():
    parser = argparse.ArgumentParser(
        description='Use this to extract thumbnails from a specific image topic of a bag')
    parser.add_argument('-i', type=str, help='input bag file', required=True)
    parser.add_argument(
        '-o', type=str,  help='output directory', required=True)
    parser.add_argument("--topic", type=str, default="/F1/image_raw",
                        help="Image topic to extract from")
    parser.add_argument("--width", type=int, default=480,
                        help="Thumbnail image width")
    parser.add_argument("--interval", type=int, default=10,
                        help="Time interval in seconds between extracted images")
    args = parser.parse_args()

    extract_images(args.i, args.topic, args.o, args.width, args.interval)


def run_from_server():
    skip_existing = False
    input_path = "/media/nick/My_Passport/datasets/ig2"
    dataset_names = os.listdir(input_path)
    dataset_names = ["2021_10_07_13_57_03_ConestogoBridge"]
    print(f"processing {len(dataset_names)} datasets")
    for dataset_name in tqdm(dataset_names):
        start_time = time.perf_counter()
        dataset_path = os.path.join(input_path, dataset_name)
        output = os.path.join(dataset_path, "thumbnails")
        if skip_existing and os.path.exists(os.path.join(output, "metadata.json")):
            print(f"skipping {dataset_name} as results exist")
        else:
            with tempfile.TemporaryDirectory() as temp_dir:
                bag_path = os.path.join(dataset_path, "raw.bag")
                tmp_bag_path = os.path.join(temp_dir, "raw.bag")
                print("copying file to local...")
                shutil.copy(bag_path, tmp_bag_path)
                extract_images(tmp_bag_path, "/F1/image_raw", output, 480, 10)

        end_time = time.perf_counter()
        print(
            f"\nFinished processing {dataset_name} in {int(end_time - start_time)} seconds!")


if __name__ == "__main__":
    # main()
    run_from_server()
