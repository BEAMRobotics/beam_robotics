import argparse
import sys
import os
import json
import logging
from typing import Any, Dict, List
import shutil
import statistics

from run_map_builder import export_corrected_poses, export_raw_slam_poses
from utils import start_ros_master, start_calibration_publisher_with_file
from params import *

logger = logging.getLogger("RUN_HELPERS")

REFINEMENT_BIN = os.path.join(
    CATKIN_WS, "devel/lib/bs_tools/bs_tools_global_map_refinement_main")
IMAGE_EXTRACTOR_BIN = os.path.join(
    CATKIN_WS, "build/inspection/inspection_extract_images")
IMAGE_SELECTOR_BIN = os.path.join(
    CATKIN_WS, "build/inspection/inspection_view_and_filter_images")
MAP_LABELER_BIN = os.path.join(
    CATKIN_WS, "build/inspection/inspection_label_map")
BIN_PATH_MAP_QUALITY = os.path.join(
    CATKIN_WS, "build/map_quality")
BIN_TRAJECTORY_VALIDATION = os.path.join(
    CATKIN_WS, "build/lidar_trajectory_validation/lidar_trajectory_validation_main")


def setup_logger():
    logger.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        '[%(levelname)s] %(asctime)s-%(name)s: %(message)s')
    handler1 = logging.StreamHandler(sys.stdout)
    handler1.setLevel(logging.DEBUG)
    handler1.setFormatter(formatter)
    logger.addHandler(handler1)


setup_logger()


def parse_args(args) -> Any:
    parser = argparse.ArgumentParser(
        description='Run full inspection pipeline')
    parser.add_argument('-d', type=int, help='dataset number')
    args = parser.parse_args()
    return args


def run_slam(datasets_config: Dict, output_path: str, dataset_number: int, rosbag_play_rate: str, local_mapper_config: str, global_mapper_config: str):
    print("\n------------------------------------")
    print("----------- Running SLAM ------------")
    print("------------------------------------\n")

    dataset_path = datasets_config["datasets"][dataset_number]["path"]
    start_time_s = datasets_config["datasets"][dataset_number]["start_time_s"]
    duration_s = datasets_config["datasets"][dataset_number]["duration_s"]

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
    cmd = f"python3 {slam_script_path} -b {bag_path} -s {start_time_s} "
    cmd += f"-d {duration_s} -r {rosbag_play_rate} -o {slam_output_path} "
    cmd += f"-local_mapper_config {local_mapper_config} "
    cmd += f"-global_mapper_config {global_mapper_config}"
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run_slam_trajectory_validation(datasets_config: Dict, output_path: str,  slam_output_path: str, dataset_number: int):
    print("\n---------------------------------------------")
    print("---- Running SLAM Trajectory Validation -----")
    print("---------------------------------------------\n")
    traj_val_output_path = os.path.join(output_path, TRAJECTORY_VAL_FOLDER)
    os.makedirs(traj_val_output_path, exist_ok=True)

    # Combine high rate and low rate trajectories
    local_mapper_bag = os.path.join(slam_output_path, LOCAL_MAPPER_BAG_FILE)
    export_raw_slam_poses(type="JSON", topic="/local_mapper/inertial_odometry/odometry",
                          bag_file=local_mapper_bag, output_dir=traj_val_output_path, prefix="local_mapper_io")

    global_map_path = os.path.join(slam_output_path, GLOBAL_MAPPER_RESULTS)
    poses_low_rate = os.path.join(
        global_map_path, "global_map_trajectory_optimized.json")
    poses_high_rate = os.path.join(
        traj_val_output_path, "local_mapper_io_poses.json")
    export_corrected_poses(type="JSON", poses_low_rate=poses_low_rate,
                           poses_high_rate=poses_high_rate, output_dir=traj_val_output_path, prefix="slam")
    slam_poses_path = os.path.join(traj_val_output_path, "slam_poses.json")

    # run validation
    dataset_path = datasets_config["datasets"][dataset_number]["path"]
    inspection_bag_path = os.path.join(dataset_path, INSPECTION_BAG_FILE)
    config_path = os.path.join(
        TRAJECTORY_VAL_INPUT, "trajectory_validation_config.json")
    timestamps_path = os.path.join(
        TRAJECTORY_VAL_INPUT, f"d{dataset_number}_timestamps.json")

    matchers = ["icp", "gicp", "ndt"]
    results_combined = {}
    rots = []
    trans = []
    for matcher in matchers:
        matcher_config = os.path.join(TRAJECTORY_VAL_INPUT, matcher + ".json")
        matcher_output = os.path.join(traj_val_output_path, matcher)
        os.makedirs(matcher_output, exist_ok=True)
        cmd = f"{BIN_TRAJECTORY_VALIDATION} -bag {inspection_bag_path} -config {config_path} "
        cmd += f"-extrinsics {EXTRINSICS_JSON_PATH} -matcher_config {matcher_config} "
        cmd += f"-output {matcher_output} -poses {slam_poses_path} -timestamps {timestamps_path} "
        logger.info("running command: %s", cmd)
        os.system(cmd)

        results_path = os.path.join(
            matcher_output, "trajectory_validation_results.json")
        with open(results_path, "r") as f:
            results = json.load(f)
            rotation = results["mean_rotation_deg"]
            if not rotation:
                continue
            translation = results["mean_translation_norm_m"]
            if not translation:
                continue
            results_combined[matcher] = {
                "mean_rotation_deg": rotation, "mean_translation_deg": translation}
            rots.append(rotation)
            trans.append(translation)
    if rots:
        results_combined["median_rotation_error_deg"] = statistics.median(rots)
    else:
        results_combined["median_rotation_error_deg"] = None
    if trans:
        results_combined["median_translation_error_m"] = statistics.median(
            trans)
    else:
        results_combined["median_translation_error_m"] = None

    results_combined_path = os.path.join(
        traj_val_output_path, TRAJ_VALIDATION_RESULTS_FILENAME)
    logger.info(f"saving results to: {results_combined_path}")
    with open(results_combined_path, 'w') as f:
        json.dump(results_combined, f, indent=4)
    logger.info("done saving results")


def run_map_refinement(output_path: str, run_submap_refinement: bool, run_submap_alignment: bool, run_posegraph_optimization: bool, run_batch_optimizer: bool):
    print("\n------------------------------------")
    print("------- Running Map Refinement ------")
    print("------------------------------------\n")

    rosmaster = start_ros_master()
    # start_calibration_publisher()
    calibration_publisher_process = start_calibration_publisher_with_file(
        EXTRINSICS_JSON_PATH)

    calibration_yaml = os.path.join(
        BS_CONFIG_FILES_PATH, "calibration_params.yaml")
    global_map_dir = os.path.join(
        output_path, "slam/global_mapper_results/GlobalMapData")
    refinement_config = os.path.join(
        BS_CONFIG_FILES_PATH, "global_map/global_map_refinement.json")
    refinement_output = os.path.join(output_path, SLAM_OUTPUT_FOLDER)
    cmd = f"{REFINEMENT_BIN} -calibration_yaml {calibration_yaml} "
    cmd += f"-globalmap_dir {global_map_dir} -output_path {refinement_output} "
    cmd += f"-refinement_config {refinement_config} -run_posegraph_optimization={run_posegraph_optimization} "
    cmd += f"-run_submap_alignment={run_submap_alignment} -run_submap_refinement={run_submap_refinement} "
    cmd += f"-run_batch_optimizer={run_batch_optimizer}"
    logger.info("running command: %s", cmd)
    os.system(cmd)
    calibration_publisher_process.stop()
    rosmaster.shutdown()


def run_map_builder(datasets_config: Dict, output_path: str, dataset_number: int, use_refined_results: bool):
    print("\n------------------------------------")
    print("------- Running Map Builder --------")
    print("------------------------------------\n")

    map_builder_output_path = os.path.join(output_path, MAP_BUILDER_FOLDER)
    if not os.path.exists(map_builder_output_path):
        logger.info("creating output directory: %s", map_builder_output_path)
        os.mkdir(map_builder_output_path)

    dataset_path = datasets_config["datasets"][dataset_number]["path"]
    inspection_bag_path = os.path.join(dataset_path, INSPECTION_BAG_FILE)
    slam_output = os.path.join(output_path, SLAM_OUTPUT_FOLDER)
    local_mapper_bag = os.path.join(slam_output, LOCAL_MAPPER_BAG_FILE)

    config_filename = datasets_config["datasets"][dataset_number]["map_builder_config_filename"]
    config_path = os.path.join(PIPELINE_INPUTS, config_filename)

    if not os.path.exists(config_path):
        logger.error(f"invalid config path")
        raise Exception("invalid config path")

    map_builder_script_path = os.path.join(
        PIPELINES_PATH, "run_map_builder.py")
    cmd = f"python3 {map_builder_script_path} -b {inspection_bag_path} -local_mapper_bag {local_mapper_bag} -c {config_path}"
    cmd += f" -o {map_builder_output_path} "
    if use_refined_results:
        cmd += "--use_refined_results"
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run_map_quality(output_path: str):
    print("\n------------------------------------")
    print("------- Running Map Quality --------")
    print("------------------------------------\n")

    bin_path = os.path.join(BIN_PATH_MAP_QUALITY,
                            "map_quality_run_map_quality_analysis")
    map_builder_output_path = os.path.join(output_path, MAP_BUILDER_FOLDER)
    map_path = os.path.join(map_builder_output_path, "map.pcd")
    output_file = os.path.join(map_builder_output_path, "map_quality.json")
    cmd = f"{bin_path} --cloud {map_path} --output {output_file}"
    cmd += "-neighborhood=false -voxel=true"
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run_image_extractor(datasets_config: str, output_path: str, dataset_number: int):
    print("\n------------------------------------")
    print("----- Running Image Extractor ------")
    print("------------------------------------\n")

    dataset_path = datasets_config["datasets"][dataset_number]["path"]
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
        f"overwriting intrinsics directory with: {INTRINSICS_PATH}")
    j["intrinsics_directory"] = INTRINSICS_PATH
    image_extractor_config_out = os.path.join(
        img_extractor_output, "image_extractor_config.json")
    with open(image_extractor_config_out, "w") as outfile:
        json.dump(j, outfile, indent=4)
    f.close()

    cmd = f"{IMAGE_EXTRACTOR_BIN} -bag {bag_path} -config {image_extractor_config_out} "
    cmd += f"-output {img_extractor_output} -poses {poses_path}"
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run_image_selection(output_path: str):
    print("\n------------------------------------")
    print("----- Running Image Selection ------")
    print("------------------------------------\n")
    img_extractor_output = os.path.join(output_path, IMAGE_EXTRACTOR_FOLDER)
    camera_list = os.path.join(img_extractor_output, "CameraList.json")

    cmd = f"{IMAGE_SELECTOR_BIN} -camera_list {camera_list} "
    cmd += f"-image_container_type IMAGE_BRIDGE "
    cmd += f"-images_filename selected_images "
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run_map_labeler(output_path: str):
    print("\n------------------------------------")
    print("------- Running Map Labeler --------")
    print("------------------------------------\n")
    image_extractor_output = os.path.join(
        output_path, IMAGE_EXTRACTOR_FOLDER)
    cameras_path = os.path.join(image_extractor_output, "CameraListNew.json")

    map_builder_output = os.path.join(output_path, MAP_BUILDER_FOLDER)
    poses_path = os.path.join(map_builder_output, "final_poses.json")
    extrinsics = os.path.join(
        EXTRINSICS_PATH, "extrinsics.json")
    config_path = os.path.join(PIPELINE_INPUTS, "map_labeler_config.json")
    cmd = f"{MAP_LABELER_BIN} -color_map=true -label_defects=false -output_camera_poses=true "
    cmd += "-output_images=true -output_individual_clouds=true -remove_unlabeled=false "
    cmd += "-save_final_map=true -draw_final_map=false "
    cmd += f"-images {cameras_path} -poses {poses_path} -config {config_path} "
    cmd += f"-intrinsics {INTRINSICS_PATH} -extrinsics {extrinsics} "

    # run for hvlp map if exists
    map_path = os.path.join(map_builder_output, "map_lidar_h_link.pcd")
    if os.path.exists(map_path):
        logger.info(f"running map labeling with horizontal lidar map")
        labeler_output_path = os.path.join(
            output_path, MAP_LABELER_FOLDER + "_hvlp")
        os.makedirs(labeler_output_path, exist_ok=True)
        cmd1 = cmd + f" -map {map_path} -output {labeler_output_path}"
        logger.info("running command: %s", cmd1)
        os.system(cmd1)

    # run for vvlp map if exists
    map_path = os.path.join(map_builder_output, "map_lidar_v_link.pcd")
    if os.path.exists(map_path):
        logger.info(f"running map labeling with vertical lidar map")
        labeler_output_path = os.path.join(
            output_path, MAP_LABELER_FOLDER + "_vvlp")
        os.makedirs(labeler_output_path, exist_ok=True)
        cmd1 = cmd + f" -map {map_path} -output {labeler_output_path}"
        logger.info("running command: %s", cmd1)
        os.system(cmd1)


def load_run_all_config():
    config_path = os.path.join(PIPELINE_INPUTS, "run_all_config.json")
    logger.info(f"loading pipeline config json from {config_path}")
    f = open(config_path)
    config = json.load(f)
    f.close()
    return config
