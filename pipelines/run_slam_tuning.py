import argparse
import sys
import os
import json
import logging
import shutil
import hashlib

from hyperparameter_tuning import HyperParamTuning
from params import *
from utils import load_datasets_config
from run_helpers import run_slam, run_slam_trajectory_validation
logger = logging.getLogger("RUN_SLAM_TUNING")

trajectory_checksum = ""
map_checksum = ""
map_builder_trajectory_checksum = ""

ROSBAG_PLAY_RATE: float = 0.5
LOCAL_MAPPER_CONFIG: str = "lio.yaml"
GLOBAL_MAPPER_CONFIG: str = "global_mapper.yaml"

SLAM_TUNING_OUTPUT = "slam_tuning"


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


def parse_args(args):
    parser = argparse.ArgumentParser(description='Run slam tuning')
    parser.add_argument('-d', type=int, help='dataset number')
    parser.add_argument(
        '-c', type=str, help='config filename for hyperparameter tuning. Should be in pipelines/inputs')
    args = parser.parse_args()
    return args


def calculate_file_sha1(file_path: str) -> str:
    sha1 = hashlib.sha1()
    with open(file_path, "rb") as file:
        chunk = file.read(4096)  # Read the file in 4KB chunks
        while chunk:
            sha1.update(chunk)
            chunk = file.read(4096)

    sha1_sum = sha1.hexdigest()
    return sha1_sum


def trajectory_exists(output_path: str) -> bool:
    slam_output_path = os.path.join(output_path, SLAM_OUTPUT_FOLDER)
    global_map_ref_path = os.path.join(
        slam_output_path, GLOBAL_MAPPER_RESULTS)
    poses_low_rate = os.path.join(
        global_map_ref_path, "global_map_trajectory_optimized.pcd")
    if not os.path.exists(poses_low_rate):
        logger.info("Slam trajectory not found!")
        return False
    return True


def has_trajectory_changed(output_path: str) -> bool:
    slam_output_path = os.path.join(output_path, SLAM_OUTPUT_FOLDER)
    global_map_ref_path = os.path.join(
        slam_output_path, GLOBAL_MAPPER_RESULTS)
    poses_low_rate = os.path.join(
        global_map_ref_path, "global_map_trajectory_optimized.pcd")
    logger.info(f"calculating checksum for trajectory file: {poses_low_rate}")
    sha1_sum = calculate_file_sha1(poses_low_rate)
    global trajectory_checksum
    logger.info(f"calculated checksum: {sha1_sum}")
    logger.info(f"previous checksum  : {trajectory_checksum}")
    if sha1_sum == trajectory_checksum:
        logger.warning(
            "trajectory hasn't changed from the last iteration. Did map refinement crash?")
        return False
    else:
        trajectory_checksum = sha1_sum
        logger.info(f"setting trajectory checksum to: {trajectory_checksum}")
        return True


def run(dataset_number: int, config_filename: str):
    datasets_config = load_datasets_config()
    if dataset_number > (len(datasets_config["datasets"]) - 1):
        logger.error("Invalid dataset_number")
        exit()

    dataset_path = datasets_config["datasets"][dataset_number]["path"]
    output_path = os.path.join(dataset_path, RESULTS_FOLDER)
    slam_tuning_output_path = os.path.join(output_path, SLAM_TUNING_OUTPUT)
    os.makedirs(slam_tuning_output_path, exist_ok=True)
    log_path = os.path.join(slam_tuning_output_path, "run_slam_tuning.log")
    setup_logger(log_path)

    config_path = os.path.join(PIPELINE_INPUTS, config_filename)
    if not os.path.exists(config_path):
        raise Exception(f"config path not found: {config_path}")

    hyperparam_output = os.path.join(
        slam_tuning_output_path, HYPER_PARAM_TUNING_FOLDER)
    os.makedirs(hyperparam_output, exist_ok=True)
    traj_val_results_path = os.path.join(
        slam_tuning_output_path, TRAJECTORY_VAL_FOLDER, TRAJ_VALIDATION_RESULTS_FILENAME)
    param_tuning = HyperParamTuning(
        slam_tuning_output_path, traj_val_results_path, config_path)
    slam_output = os.path.join(output_path, SLAM_OUTPUT_FOLDER)
    while param_tuning.next():
        run_slam(datasets_config, output_path, dataset_number, ROSBAG_PLAY_RATE,
                 LOCAL_MAPPER_CONFIG, GLOBAL_MAPPER_CONFIG)

        if not trajectory_exists(output_path) or not has_trajectory_changed(output_path):
            # try one more time:
            logger.info("looks like slam failed! Trying one more time...")
            run_slam(datasets_config, output_path, dataset_number, ROSBAG_PLAY_RATE,
                     LOCAL_MAPPER_CONFIG, GLOBAL_MAPPER_CONFIG)
            if not trajectory_exists(output_path) or not has_trajectory_changed(output_path):
                param_tuning.mark_as_failed()
                continue

        run_slam_trajectory_validation(
            datasets_config, slam_tuning_output_path, slam_output, dataset_number)

        # copy results
        src = os.path.join(
            slam_tuning_output_path, TRAJECTORY_VAL_FOLDER)
        dst = os.path.join(hyperparam_output,
                           f"iter_{param_tuning.get_iteration()}")
        shutil.copytree(src, dst)

        # copy these config files:
        files = [
            os.path.join(BS_CONFIG_FILES_PATH, "matchers", "loam_vlp16.json"),
            os.path.join(BS_CONFIG_FILES_PATH,
                         "registration", "scan_to_map.json")
        ]
        for file in files:
            filename = os.path.basename(file)
            shutil.copyfile(file, os.path.join(dst, filename))

    param_tuning.store_best_parameter(
        parameter_key="median_translation_error_m")
    param_tuning.cleanup()

    logger.info("slam tuning pipeline completed successfully")


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])
    run(args.d, args.c)
