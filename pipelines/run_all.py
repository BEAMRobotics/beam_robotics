import argparse
import sys
import os
import json
import logging
from typing import Any, Dict
import shutil
import hashlib

from utils import start_ros_master, start_calibration_publisher_with_file
from hyperparameter_tuning import HyperParamTuning
from params import *

logger = logging.getLogger("RUN_ALL")

REFINEMENT_BIN = os.path.join(
    CATKIN_WS, "devel/lib/bs_tools/bs_tools_global_map_refinement_main")
IMAGE_EXTRACTOR_BIN = os.path.join(
    CATKIN_WS, "build/inspection/inspection_extract_images")
IMAGE_SELECTOR_BIN = os.path.join(
    CATKIN_WS, "build/inspection/inspection_view_and_filter_images")
MAP_LABELER_BIN = os.path.join(
    CATKIN_WS, "build/inspection/inspection_label_map")
BIN_PATH_MAP_QUALITY = "/userhome/catkin_ws/build/map_quality"

trajectory_checksum = ""
map_checksum = ""
map_builder_trajectory_checksum = ""


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
    duration_s = config["datasets"][dataset_number]["duration_s"]

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
    cmd = "python3 {} -b {} -s {} -d {} -r {} -o {} -local_mapper_config {} -global_mapper_config {}".format(
        slam_script_path, bag_path, start_time_s, duration_s, rate,
        slam_output_path, local_mapper_config, global_mapper_config
    )
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run_map_refinement(config: Dict, output_path: str):
    run_submap_refinement = config["map_refinement"]["run_submap_refinement"]
    run_submap_alignment = config["map_refinement"]["run_submap_alignment"]
    run_posegraph_optimization = config["map_refinement"]["run_posegraph_optimization"]
    run_batch_optimizer = config["map_refinement"]["run_batch_optimizer"]
    if not (run_posegraph_optimization or run_submap_alignment or run_submap_refinement or run_batch_optimizer):
        logger.info("skipping map refinement")
        return

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
    inspection_bag_path = os.path.join(dataset_path, INSPECTION_BAG_FILE)
    slam_output = os.path.join(output_path, SLAM_OUTPUT_FOLDER)
    local_mapper_bag = os.path.join(slam_output, LOCAL_MAPPER_BAG_FILE)

    config_filename = config["datasets"][dataset_number]["map_builder_config_filename"]
    config_path = os.path.join(PIPELINE_INPUTS, config_filename)
    if len(config_filename) == 0 or config_filename == "DEFAULT":
        config_path = os.path.join(
            PIPELINE_INPUTS, "map_builder_config_default.json")
        logger.info(f"Using default map builder config file: {config_path}")
    elif config_filename == "HANDHELD":
        config_path = os.path.join(
            PIPELINE_INPUTS, "map_builder_config_handheld.json")
        logger.info(f"Using handheld map builder config file: {config_path}")
    else:
        logger.info(f"Using custom map builder config file: {config_path}")

    if not os.path.exists(config_path):
        logger.error(f"invalid config path")
        raise Exception("invalid config path")

    map_builder_script_path = os.path.join(
        PIPELINES_PATH, "run_map_builder.py")
    cmd = f"python3 {map_builder_script_path} -b {inspection_bag_path} -local_mapper_bag {local_mapper_bag} -c {config_path}"
    cmd += f" -o {map_builder_output_path}"
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run_map_quality(config: Dict, output_path: str):
    if not config["run_map_quality"]:
        logger.info("skipping map quality")
        return

    print("\n------------------------------------")
    print("------- Running Map Quality --------")
    print("------------------------------------\n")

    bin_path = os.path.join(BIN_PATH_MAP_QUALITY,
                            "map_quality_run_map_quality_analysis")
    map_builder_output_path = os.path.join(output_path, MAP_BUILDER_FOLDER)
    map_path = os.path.join(map_builder_output_path, "map.pcd")
    output_file = os.path.join(map_builder_output_path, "map_quality.json")
    cmd = "{} --cloud {} --output {}".format(bin_path, map_path, output_file)
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run_image_extractor(config: str, output_path: str, dataset_number: int):
    if not config["run_image_extractor"]:
        logger.info("skipping image extractor")
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


def run_image_selection(config: str, output_path: str):
    img_extractor_output = os.path.join(output_path, IMAGE_EXTRACTOR_FOLDER)
    camera_list = os.path.join(img_extractor_output, "CameraList.json")

    if not config["run_image_selection"]:
        logger.info("skipping image selection")
        new_list_file = os.path.join(
            img_extractor_output, "CameraListNew.json")
        if not os.path.exists(new_list_file) and os.path.exists(camera_list):
            logger.info(
                f"no CameraListNew file, copying from {camera_list} to {new_list_file}")
            shutil.copyfile(camera_list, new_list_file)
        return

    print("\n------------------------------------")
    print("----- Running Image Selection ------")
    print("------------------------------------\n")

    cmd = f"{IMAGE_SELECTOR_BIN} -camera_list {camera_list} "
    cmd += f"-image_container_type IMAGE_BRIDGE "
    cmd += f"-images_filename selected_images "
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run_map_labeler(config: str, output_path: str):
    if not config["run_map_labeler"]:
        logger.info("skipping map labeler")
        return

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


def run(dataset_number: int):
    config = load_config()
    if dataset_number > (len(config["datasets"]) - 1):
        logger.error("Invalid dataset_number")
        exit()

    dataset_path = config["datasets"][dataset_number]["path"]
    output_path = os.path.join(dataset_path, RESULTS_FOLDER)

    log_path = os.path.join(output_path, "run_all_pipeline.log")
    setup_logger(log_path)

    if not os.path.exists(output_path):
        logger.info("creating output directory: %s", output_path)
        os.mkdir(output_path)

    if config["run_hyperparameter_tuning"]:
        if not config["run_map_quality"]:
            logger.warning(
                "Turning on run_map_quality which is needed for Hyper Parameter Tuning")
            config["run_map_quality"] = True
        if not config["run_map_builder"]:
            logger.warning(
                "Turning on run_map_builder which is needed for Hyper Parameter Tuning")
            config["run_map_builder"] = True
        hyperparam_output = os.path.join(
            output_path, HYPER_PARAM_TUNING_FOLDER)
        os.makedirs(hyperparam_output, exist_ok=True)
        param_tuning = HyperParamTuning(output_path)

        while param_tuning.next():
            run_slam(config, output_path, dataset_number)
            run_map_refinement(config, output_path)
            if not has_trajectory_changed(output_path):
                param_tuning.mark_as_failed()
                continue
            run_map_builder(config, output_path, dataset_number)
            if not has_map_trajectory_changed(output_path) or not has_map_changed(output_path):
                param_tuning.mark_as_failed()
                continue
                # print("still going on")
            run_map_quality(config, output_path)

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
        param_tuning.set_best_parameters()

    run_slam(config, output_path, dataset_number)
    run_map_refinement(config, output_path)
    run_map_builder(config, output_path, dataset_number)
    run_map_quality(config, output_path)
    run_image_extractor(config, output_path, dataset_number)
    run_image_selection(config, output_path)
    run_map_labeler(config, output_path)
    if config["run_hyperparameter_tuning"]:
        param_tuning.cleanup()

    logger.info("run_all.py pipeline completed successfully")


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])
    run(args.d)
