import argparse
import sys
import os
from typing import Any
import logging

from params import *

logger = logging.getLogger("RECALCULATE_MAP_QUALITY")
BIN_PATH = "/userhome/catkin_ws/build/map_quality/map_quality_run_map_quality_analysis"
DATA_PATH = "/userhome/data"


def setup_logger(output_file: str):
    logger.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        '[%(levelname)s] %(asctime)s-%(name)s: %(message)s')
    handler1 = logging.StreamHandler(sys.stdout)
    handler1.setLevel(logging.DEBUG)
    handler1.setFormatter(formatter)
    logger.addHandler(handler1)
    if output_file:
        handler2 = logging.FileHandler(output_file)
        handler2.setLevel(logging.DEBUG)
        handler2.setFormatter(formatter)
        logger.addHandler(handler2)


def parse_args(args) -> Any:
    parser = argparse.ArgumentParser(description='run_cam_to_map_calibrator')
    parser.add_argument(
        '-d', type=str, help='Dataset name. Should be the folder in /userhome/data')
    parser.add_argument(
        '-c', type=str, help='Camera name. E.g., F1')
    parser.add_argument('-l', type=str, help='Lidar frame')
    args = parser.parse_args()
    return args


def run(dataset_path: str, output_path: str, camera_name: str, lidar_link: str):
    image_extractor_path = os.path.join(dataset_path, IMAGE_EXTRACTOR_FOLDER)
    if not os.path.exists(image_extractor_path):
        raise Exception(
            f"No image extractor path found, did you run extract images? Path: {image_extractor_path}")
    images_path = os.path.join(image_extractor_path, camera_name + "_link")
    if not os.path.exists(images_path):
        raise Exception(f"Images path not found: {images_path}")
    map_path = os.path.join(dataset_path, MAP_BUILDER_FOLDER, MAP_FILENAME)
    poses_path = os.path.join(dataset_path, MAP_BUILDER_FOLDER, POSES_FILENAME)
    exintrics_path = os.path.join(
        EXTRINSICS_PATH, "extrinsics.json")
    intrinsics_path = os.path.join(
        INTRINSICS_PATH, camera_name + ".json")

    cmd = f"{BIN_PATH} -extrinsics {exintrics_path} "
    cmd += f"-intrinsics {intrinsics_path} "
    cmd += f"-images_list {os.path.join(images_path, IMAGES_LIST_FILENAME)} "
    cmd += f"-map {map_path} "
    cmd += f"-map_sensor_frame {lidar_link} "
    cmd += f"-poses {poses_path} "
    cmd += f"-output {os.path.join(output_path, OUTPUT_FILENAME)} "
    logger.info(f"Running command: {cmd}")
    os.system(cmd)
    logger.info("run_cam_to_map_calibrator.py finished successfully!")


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])

    dataset_path = os.path.join(DATA_PATH, args.d, "results")
    if not os.path.exists(dataset_path):
        raise Exception(f"dataset_path not found: {dataset_path}")
    output_path = os.path.join(dataset_path, "cam_to_map_calibrator")
    os.makedirs(output_path, exist_ok=True)
    setup_logger(os.path.join(output_path, "run_cam_to_map_calibrator.log"))
    run(dataset_path, output_path, args.c, args.l)
