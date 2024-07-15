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
from run_helpers import run_slam, run_map_refinement, run_map_builder, run_map_quality
logger = logging.getLogger("RUN_HYPERPARAM_TUNING")

trajectory_checksum = ""
map_checksum = ""
map_builder_trajectory_checksum = ""

ROSBAG_PLAY_RATE = 0.25
LOCAL_MAPPER_CONFIG = "lio.yaml",
GLOBAL_MAPPER_CONFIG = "global_mapper.yaml"

RUN_SLAM = False
RUN_SUBMAP_REFINEMENT = False
RUN_SUBMAP_ALIGNMENT = False
RUN_PGO = False
RUN_BATCH = True


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
    parser = argparse.ArgumentParser(
        description='Run hyperparamter tuning using full inspection pipeline & map quality')
    parser.add_argument('-d', type=int, help='dataset number')
    parser.add_argument(
        '-c', type=int, help='config filename for hyperparameter tuning. Should be in pipelines/inputs')
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


def has_trajectory_changed(output_path: str) -> bool:
    slam_output_path = os.path.join(output_path, SLAM_OUTPUT_FOLDER)
    global_map_ref_path = os.path.join(
        slam_output_path, GLOBAL_MAP_REFINEMENT_RESULTS)
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


def has_map_trajectory_changed(output_path: str) -> bool:
    path = os.path.join(output_path, MAP_BUILDER_FOLDER, "final_poses.pcd")
    logger.info(
        f"calculating checksum for map builder trajectory file: {path}")
    sha1_sum = calculate_file_sha1(path)
    global map_builder_trajectory_checksum
    logger.info(f"calculated checksum: {sha1_sum}")
    logger.info(f"previous checksum  : {map_builder_trajectory_checksum}")
    if sha1_sum == map_builder_trajectory_checksum:
        logger.warning(
            "map builder trajectory hasn't changed from the last iteration. Did map builder crash?")
        return False
    else:
        map_builder_trajectory_checksum = sha1_sum
        logger.info(
            f"setting map builder trajectory checksum to: {map_builder_trajectory_checksum}")
        return True


def has_map_changed(output_path: str) -> bool:
    map_path = os.path.join(output_path, MAP_BUILDER_FOLDER, "map.pcd")
    logger.info(f"calculating checksum for map file: {map_path}")
    sha1_sum = calculate_file_sha1(map_path)
    global map_checksum
    logger.info(f"calculated checksum: {sha1_sum}")
    logger.info(f"previous checksum  : {map_checksum}")
    if sha1_sum == map_checksum:
        logger.warning(
            "map hasn't changed from the last iteration. Did map builder crash?")
        return False
    else:
        map_checksum = sha1_sum
        logger.info(f"setting map checksum to: {map_checksum}")
        return True


def run(dataset_number: int, config_filename: str):
    datasets_config = load_datasets_config()
    if dataset_number > (len(datasets_config["datasets"]) - 1):
        logger.error("Invalid dataset_number")
        exit()

    dataset_path = datasets_config["datasets"][dataset_number]["path"]
    output_path = os.path.join(dataset_path, RESULTS_FOLDER)

    log_path = os.path.join(output_path, "run_hyperparameter_tuning.log")
    setup_logger(log_path)

    if not os.path.exists(output_path):
        logger.info("creating output directory: %s", output_path)
        os.mkdir(output_path)

    config_path = os.path.join(PIPELINE_INPUTS, config_filename)
    if os.path.exists(config_path):
        raise Exception(f"config path not found: {config_path}")

    hyperparam_output = os.path.join(output_path, HYPER_PARAM_TUNING_FOLDER)
    os.makedirs(hyperparam_output, exist_ok=True)
    map_quality_results_path = os.path.join(
        output_path, MAP_BUILDER_FOLDER, MAP_QUALITY_FILENAME)
    param_tuning = HyperParamTuning(
        output_path, map_quality_results_path, config_path)

    while param_tuning.next():
        # slam
        if RUN_SLAM:
            run_slam(datasets_config, output_path, dataset_number, ROSBAG_PLAY_RATE,
                     LOCAL_MAPPER_CONFIG, GLOBAL_MAPPER_CONFIG)
        else:
            logger.info("skipping slam")

        run_map_refinement(output_path, RUN_SUBMAP_REFINEMENT,
                           RUN_SUBMAP_ALIGNMENT, RUN_PGO, RUN_BATCH)

        if not has_trajectory_changed(output_path):
            param_tuning.mark_as_failed()
            continue

        run_map_builder(datasets_config, output_path, dataset_number)
        if not has_map_trajectory_changed(output_path) or not has_map_changed(output_path):
            param_tuning.mark_as_failed()
            continue

        run_map_quality(output_path)

        # copy trajectory and map files for reference
        src = os.path.join(
            output_path, MAP_BUILDER_FOLDER, "map_quality.json")
        dst = os.path.join(
            hyperparam_output, f"map_quality_{param_tuning.get_iteration()}.json")
        shutil.copyfile(src, dst)
        src = os.path.join(output_path, MAP_BUILDER_FOLDER, "map.pcd")
        dst = os.path.join(hyperparam_output,
                           f"map_{param_tuning.get_iteration()}.pcd")
        shutil.copyfile(src, dst)
        src = os.path.join(
            output_path, MAP_BUILDER_FOLDER, "final_poses.json")
        dst = os.path.join(
            hyperparam_output, f"final_poses{param_tuning.get_iteration()}.json")
        shutil.copyfile(src, dst)
        src = os.path.join(
            output_path, MAP_BUILDER_FOLDER, "final_poses.pcd")
        dst = os.path.join(
            hyperparam_output, f"final_poses{param_tuning.get_iteration()}.pcd")
        shutil.copyfile(src, dst)
    param_tuning.store_best_parameter()
    param_tuning.cleanup()

    logger.info("hyper parameter tuning pipeline completed successfully")


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])
    run(args.d, args.c)
