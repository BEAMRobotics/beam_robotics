import argparse
import sys
import os
import json
import logging
from typing import Any, Dict
import shutil

from utils import start_ros_master, start_calibration_publisher
from params import *

logger = logging.getLogger("RUN_ALL")

REFINEMENT_BIN = os.path.join(
    CATKIN_WS, "devel/lib/bs_tools/bs_tools_global_map_refinement_main")
IMAGE_EXTRACTOR_BIN = os.path.join(
    CATKIN_WS, "build/inspection/inspection_extract_images")


def setup_logger():
    logger.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        '[%(levelname)s] %(asctime)s-%(name)s: %(message)s')
    handler1 = logging.StreamHandler(sys.stdout)
    handler1.setLevel(logging.DEBUG)
    handler1.setFormatter(formatter)
    logger.addHandler(handler1)


def parse_args(args) -> Any:
    parser = argparse.ArgumentParser(
        description='Run full inspection pipeline')
    parser.add_argument('-d', type=int, help='dataset number')
    args = parser.parse_args()
    return args


def load_config() -> Any:
    config_path = os.path.join(PIPELINE_INPUTS, "datasets_config.json")
    logger.info(f"loading pipeline config json from {config_path}")
    f = open(config_path)
    config = json.load(f)
    f.close()
    return config


def run_slam(config: Dict, output_path: str, dataset_number: int):
    if not config["run_slam"]:
        logger.info("skipping slam")
        return

    print("\n------------------------------------")
    print("----------- Running SLAM ------------")
    print("------------------------------------\n")

    rate = config["rosbag_play_rate"]
    local_mapper_config = config["local_mapper_config"]
    global_mapper_config = config["global_mapper_config"]
    dataset_path = config["datasets"][dataset_number]["path"]
    start_time_s = config["datasets"][dataset_number]["start_time_s"]
    end_time_s = config["datasets"][dataset_number]["end_time_s"]

    logger.info("processing dataset: %s", dataset_path)

    bag_path = os.path.join(dataset_path, SLAM_BAG_FILE)

    if not os.path.exists(bag_path):
        logger.error(
            f"invalid dataset path, no {SLAM_BAG_FILE} file in: {dataset_path}")
        exit()

    slam_output_path = os.path.join(output_path, SLAM_OUTPUT_FOLDER)
    if not os.path.exists(slam_output_path):
        logger.info("creating output directory: %s", slam_output_path)
        os.mkdir(slam_output_path)

    slam_script_path = os.path.join(PIPELINES_PATH, "run_beam_slam.py")
    cmd = "python3 {} -b {} -s {} -e {} -r {} -o {} -local_mapper_config {} -global_mapper_config {}".format(
        slam_script_path, bag_path, start_time_s, end_time_s, rate,
        slam_output_path, local_mapper_config, global_mapper_config
    )
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run_map_refinement(config: Dict, output_path: str):
    if not config["run_map_refinement"]:
        logger.info("skipping map refinement")
        return

    print("\n------------------------------------")
    print("------- Running Map Refinement ------")
    print("------------------------------------\n")

    rosmaster = start_ros_master()
    start_calibration_publisher()

    calibration_yaml = os.path.join(
        BS_CONFIG_FILES_PATH, "calibration_params.yaml")
    global_map_dir = os.path.join(
        output_path, "slam/global_mapper_results/GlobalMapData")
    refinement_config = os.path.join(
        BS_CONFIG_FILES_PATH, "global_map/global_map_refinement.json")
    refinement_output = os.path.join(output_path, SLAM_OUTPUT_FOLDER)
    cmd = f"{REFINEMENT_BIN} -calibration_yaml {calibration_yaml} "
    cmd += f"-globalmap_dir {global_map_dir} -output_path {refinement_output} "
    cmd += f"-refinement_config {refinement_config} -run_posegraph_optimization=true "
    cmd += "-run_submap_alignment=true -run_submap_refinement=true"
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run_map_builder(config: Dict, output_path: str, dataset_number: int):
    if not config["run_map_builder"]:
        logger.info("skipping map builder")
        return

    print("\n------------------------------------")
    print("------- Running Map Builder --------")
    print("------------------------------------\n")

    map_builder_output_path = os.path.join(output_path, MAP_BUILDER_FOLDER)
    if not os.path.exists(map_builder_output_path):
        logger.info("creating output directory: %s", map_builder_output_path)
        os.mkdir(map_builder_output_path)

    dataset_path = config["datasets"][dataset_number]["path"]
    raw_bag_path = os.path.join(dataset_path, SLAM_BAG_FILE)
    slam_output = os.path.join(output_path, SLAM_OUTPUT_FOLDER)
    local_mapper_bag = os.path.join(slam_output, LOCAL_MAPPER_BAG_FILE)

    map_builder_script_path = os.path.join(
        PIPELINES_PATH, "run_map_builder.py")
    cmd = f"python3 {map_builder_script_path} -b {raw_bag_path} -local_mapper_bag {local_mapper_bag} "
    cmd += f" -o {map_builder_output_path}"
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run_image_extractor(config: str, output_path: str, dataset_number: int):
    if not config["run_image_extractor"]:
        logger.info("skipping map builder")
        return

    print("\n------------------------------------")
    print("----- Running Image Extractor ------")
    print("------------------------------------\n")

    dataset_path = config["datasets"][dataset_number]["path"]
    bag_path = os.path.join(dataset_path, INSPECTION_BAG_FILE)
    if not os.path.exists(bag_path):
        logger.error(
            f"missing  {INSPECTION_BAG_FILE} file in: P{dataset_path}")
        exit()

    img_extractor_output = os.path.join(output_path, IMAGE_EXTRACTOR_FOLDER)
    map_builder_path = os.path.join(output_path, MAP_BUILDER_FOLDER)
    poses_path = os.path.join(map_builder_path, "final_poses.json")

    if (os.path.exists(img_extractor_output)):
        shutil.rmtree(img_extractor_output)
    os.makedirs(img_extractor_output)

    # Loam image extractor and override intrinsics directory
    image_extractor_config_in = os.path.join(
        PIPELINE_INPUTS, "image_extractor_config.json")
    logger.info(f"loading image extractor config: {image_extractor_config_in}")
    f = open(image_extractor_config_in)
    j = json.load(f)
    logger.info(
        f"overwriting intrinsics directory with: {INSPECTION_INTRINSICS_PATH}")
    j["intrinsics_directory"] = INSPECTION_INTRINSICS_PATH
    image_extractor_config_out = os.path.join(
        img_extractor_output, "image_extractor_config.json")
    with open(image_extractor_config_out, "w") as outfile:
        json.dump(j, outfile)
    f.close()

    cmd = f"{IMAGE_EXTRACTOR_BIN} -bag {bag_path} -config {image_extractor_config_out} "
    cmd += f"-output {img_extractor_output} -poses {poses_path}"
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run(dataset_number: int):
    config = load_config()
    if dataset_number > (len(config["datasets"]) - 1):
        logger.error("Invalid dataset_number")
        exit()

    dataset_path = config["datasets"][dataset_number]["path"]
    output_path = os.path.join(dataset_path, RESULTS_FOLDER)
    if not os.path.exists(output_path):
        logger.info("creating output directory: %s", output_path)
        os.mkdir(output_path)

    run_slam(config, output_path, dataset_number)
    run_map_refinement(config, output_path)
    run_map_builder(config, output_path, dataset_number)
    run_image_extractor(config, output_path, dataset_number)

    logger.info("run_all.py pipeline completed successfully")


if __name__ == "__main__":
    setup_logger()
    args = parse_args(sys.argv[1:])
    run(args.d)
