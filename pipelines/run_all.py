import argparse
import sys
import os
import json
import logging
from typing import Any
from pathlib import Path

logger = logging.getLogger("RUN_ALL")
current_file = os.path.abspath(__file__)
current_path = Path(current_file).parent

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
    parser.add_argument('-d', type=int, help='dataset number')
    parser.add_argument('-skip_slam', type=bool, 
                        help='set to true to skip slam pipeline', 
                        default=False)
    args = parser.parse_args()
    return args

def load_config() -> Any:
    inputs_dir = os.path.join(current_path, "inputs")
    config_path = os.path.join(inputs_dir, "datasets_config.json")
    f = open(config_path)
    config = json.load(f)
    f.close()
    return config
    

def run(dataset_number: int, skip_slam: bool):
    config = load_config()
    if dataset_number > (len(config["datasets"]) - 1):
        logger.error("Invalid dataset_number")
        exit()

    rate = config["rosbag_play_rate"]
    slam_config = config["slam_config"]
    dataset_path = config["datasets"][dataset_number]["path"]
    start_time_s = config["datasets"][dataset_number]["start_time_s"]
    end_time_s = config["datasets"][dataset_number]["end_time_s"]
    logger.info("processing dataset: %s", dataset_path)
    
    bag_path = os.path.join(dataset_path, "data.bag")

    if not os.path.exists(bag_path):
        logger.error("invalid dataset path, no data.bag file in: %s", dataset_path)
        exit()
    
    output_path = os.path.join(dataset_path, "results")
    slam_output_path = os.path.join(output_path, "slam")
    map_builder_output_path = os.path.join(output_path, "map_builder")

    if not os.path.exists(output_path):
        logger.info("creating output directory: %s", output_path)
        os.mkdir(output_path)

    if not os.path.exists(slam_output_path):
        logger.info("creating output directory: %s", slam_output_path)
        os.mkdir(slam_output_path)    

    if not os.path.exists(map_builder_output_path):
        logger.info("creating output directory: %s", map_builder_output_path)
        os.mkdir(map_builder_output_path)

    if skip_slam:
        logger.info("skipping slam")
    else:    
        slam_script_path = os.path.join(current_path, "run_beam_slam.py")
        cmd_slam = "python3 {} -b {} -s {} -e {} -r {} -o {} -slam_config {}".format(
            slam_script_path, bag_path, start_time_s, end_time_s, rate, 
            slam_output_path, slam_config
        )
        logger.info("running command: %s", cmd_slam)
        os.system(cmd_slam)

    map_builder_script_path = os.path.join(current_path, "run_map_builder.py")
    cmd_map_builder = "python3 {} -b {} -slam_output {} -o {}".format(
        map_builder_script_path, bag_path, slam_output_path, map_builder_output_path + "/"
    )
    logger.info("running command: %s", cmd_map_builder)
    os.system(cmd_map_builder)
    logger.info("run_all.py pipeline completed successfully")


if __name__ == "__main__":
    setup_logger()
    args = parse_args(sys.argv[1:])
    run(args.d, args.skip_slam)
