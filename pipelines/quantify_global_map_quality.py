import argparse
import sys
import os
import json
from typing import Any, List
import logging
from pathlib import Path

logger = logging.getLogger("QUANTIFY_GM_QUALITY")

BIN_PATH = "$HOME/catkin_ws/build/map_quality/map_quality_run_map_quality_analysis"
results_postfix = "_quality.json"
submap_combined_filename = "submaps_combined.pcd"


def setup_logger():
    logger.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        '[%(levelname)s] %(asctime)s-%(name)s: %(message)s')
    handler1 = logging.StreamHandler(sys.stdout)
    handler1.setLevel(logging.DEBUG)
    handler1.setFormatter(formatter)
    logger.addHandler(handler1)


def parse_args(args) -> Any:
    parser = argparse.ArgumentParser(description='Run SLAM')
    parser.add_argument(
        '-i', type=str, help='path global map output directory. '
        'This should contain lidar_submaps_initial, and lidar_submaps_optimized')
    args = parser.parse_args()
    return args


def process_single_cloud(pcd_file: str, results_file: str):
    logger.info("running map quality on pcd %s", pcd_file)
    cmd = BIN_PATH + " -cloud " + pcd_file + " -output " + results_file
    logger.info("running command: %s", cmd)
    os.system(cmd)


def combine_results(results_files: List[str], combined_json_path: str):
    results_combined_dict = {}
    for results_file in results_files:
        f = open(results_file)
        data = json.load(f)
        postfix_size = len(results_postfix)
        submap_name = os.path.basename(results_file)[:-postfix_size]
        results_combined_dict[submap_name] = data
        f.close()

    json_object = json.dumps(results_combined_dict, indent=4)
    with open(combined_json_path, "w") as outfile:
        outfile.write(json_object)


def run_quantification(submaps_path: str):
    results_files = []
    output_path = os.path.join(submaps_path, "map_quality_results")
    os.makedirs(output_path, exist_ok=True)
    for (root, dirs, files) in os.walk(submaps_path):
        for file in files:
            pcd_filepath = os.path.join(submaps_path, file)
            if Path(file).suffix != ".pcd":
                logger.info("skipping non-pcd file: %s", file)
                continue
            results_filepath = os.path.join(
                output_path, file[:-4] + results_postfix)
            results_files.append(results_filepath)
            process_single_cloud(pcd_filepath, results_filepath)
        break
    combined_json_path = os.path.join(submaps_path, "map_quality_summary.json")
    combine_results(results_files, combined_json_path)


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])

    if not os.path.exists(args.i):
        logger.error(
            "input path does not exist, exiting. Input: %s", args.i)
        exit()
    setup_logger()
    submaps_initial = os.path.join(args.i, "lidar_submaps_initial")
    submaps_opt = os.path.join(args.i, "lidar_submaps_optimized")
    run_quantification(submaps_initial)
    run_quantification(submaps_opt)
