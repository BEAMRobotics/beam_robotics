import argparse
import sys
import os
from typing import Any
import roslaunch
import logging

from params import *

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
logger = logging.getLogger("RUN_MAP_BUILDER")
BIN_PATH_MAP_BUILDER = "/userhome/catkin_ws/devel/lib/map_builder"
BIN_PATH_MAP_QUALITY = "/userhome/catkin_ws/build/map_quality"


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
    parser = argparse.ArgumentParser(description='Run SLAM')
    parser.add_argument(
        '-b', type=str, help='input bag file which contains the raw lidar data')
    parser.add_argument('-local_mapper_bag', type=str,
                        help='path to bag which contains local mapper trajectories.')
    parser.add_argument('-o', type=str, help='full path to output directory')
    args = parser.parse_args()
    return args


def export_raw_slam_poses(type: str, topic: str, bag_file: str, output_dir: str, prefix: str):
    output_path = output_dir
    if output_dir[len(output_dir) - 1] != '/':
        output_path += '/'

    bag_to_poses_bin = os.path.join(
        BIN_PATH_MAP_BUILDER, "map_builder_bag_to_poses_file")
    cmd = f"{bag_to_poses_bin} -bag {bag_file} -output_path {output_path} "
    cmd += f"-topic {topic} -output_type {type}"
    logger.info("running command: %s", cmd)
    os.system(cmd)
    if type == "PCD":
        p1 = os.path.join(output_path, "poses.pcd")
        if not os.path.exists(p1):
            logger.warning("no poses file generated for %s", prefix)
            return
        p2 = os.path.join(output_path, prefix + "_poses.pcd")
        logger.info("renaming %s to %s", p1, p2)
        os.system("mv " + p1 + " " + p2)
    elif type == "JSON":
        p1 = os.path.join(output_path, "poses.json")
        if not os.path.exists(p1):
            logger.warning("no poses file generated for %s", prefix)
            return
        p2 = os.path.join(output_path, prefix + "_poses.json")
        logger.info("renaming %s to %s", p1, p2)
        os.system("mv " + p1 + " " + p2)


def export_corrected_poses(type: str, poses_low_rate: str, poses_high_rate: str, output_dir: str, prefix: str):
    output_path = output_dir
    if output_dir[len(output_dir) - 1] != '/':
        output_path += '/'

    bag_to_poses_bin = os.path.join(
        BIN_PATH_MAP_BUILDER, "map_builder_fill_in_trajectory")
    cmd = "{} -output_path {} -poses_high_rate {} -poses_low_rate {} -output_type {}".format(
        bag_to_poses_bin, output_path,  poses_high_rate, poses_low_rate, type)
    logger.info("running command: %s", cmd)
    os.system(cmd)
    if type == "PCD":
        p1 = os.path.join(output_path, "poses.pcd")
        if not os.path.exists(p1):
            logger.warning("no poses file generated for %s", prefix)
            return
        p2 = os.path.join(output_path, prefix + "_poses.pcd")
        logger.info("renaming %s to %s", p1, p2)
        os.system("mv " + p1 + " " + p2)
    elif type == "JSON":
        p1 = os.path.join(output_path, "poses.json")
        if not os.path.exists(p1):
            logger.warning("no poses file generated for %s", prefix)
            return
        p2 = os.path.join(output_path, prefix + "_poses.json")
        logger.info("renaming %s to %s", p1, p2)
        os.system("mv " + p1 + " " + p2)


def build_map(poses_path: str, bag_file: str, output_dir: str):
    build_map_bin = os.path.join(BIN_PATH_MAP_BUILDER, "map_builder_build_map")
    config_path = os.path.join(PIPELINE_INPUTS, "map_builder_config.json")
    cmd = "{} --bag_file {} --config_file {} --extrinsics {} --output_directory {} --pose_file {}".format(
        build_map_bin, bag_file, config_path, EXTRINSICS_PATH, output_dir, poses_path)
    logger.info("Running command: %s", cmd)
    os.system(cmd)


def run_map_quality_analysis(map_path: str, output_file: str):
    bin_path = os.path.join(BIN_PATH_MAP_QUALITY,
                            "map_quality_run_map_quality_analysis")
    cmd = "{} --cloud {} --output {}".format(bin_path, map_path, output_file)
    logger.info("running command: %s", cmd)
    os.system(cmd)


def run(bag_file: str, local_mapper_bag: str, output_dir: str):
    export_raw_slam_poses(type="JSON", topic="/local_mapper/graph_publisher/odom",
                          bag_file=local_mapper_bag, output_dir=output_dir, prefix="local_mapper_graph")
    export_raw_slam_poses(type="JSON", topic="/local_mapper/inertial_odometry/odometry",
                          bag_file=local_mapper_bag, output_dir=output_dir, prefix="local_mapper_io")
    export_raw_slam_poses(type="JSON", topic="/local_mapper/lidar_odometry/odometry",
                          bag_file=local_mapper_bag, output_dir=output_dir, prefix="local_mapper_lo")
    export_raw_slam_poses(type="JSON", topic="/local_mapper/visual_odometry/odometry",
                          bag_file=local_mapper_bag, output_dir=output_dir, prefix="local_mapper_vo")

    export_raw_slam_poses(type="PCD", topic="/local_mapper/graph_publisher/odom",
                          bag_file=local_mapper_bag, output_dir=output_dir, prefix="local_mapper_graph")
    export_raw_slam_poses(type="PCD", topic="/local_mapper/inertial_odometry/odometry",
                          bag_file=local_mapper_bag, output_dir=output_dir, prefix="local_mapper_io")
    export_raw_slam_poses(type="PCD", topic="/local_mapper/lidar_odometry/odometry",
                          bag_file=local_mapper_bag, output_dir=output_dir, prefix="local_mapper_lo")
    export_raw_slam_poses(type="PCD", topic="/local_mapper/visual_odometry/odometry",
                          bag_file=local_mapper_bag, output_dir=output_dir, prefix="local_mapper_vo")

    slam_output_path = Path(local_mapper_bag).parent
    global_map_ref_path = os.path.join(
        slam_output_path, GLOBAL_MAP_REFINEMENT_RESULTS)
    poses_low_rate = os.path.join(
        global_map_ref_path, "global_map_trajectory_optimized.json")

    poses_high_rate = os.path.join(output_dir, "local_mapper_io_poses.json")
    export_corrected_poses("PCD", poses_low_rate,
                           poses_high_rate, output_dir, "final")
    export_corrected_poses("JSON", poses_low_rate,
                           poses_high_rate, output_dir, "final")

    final_poses_path = os.path.join(output_dir, "final_poses.json")
    build_map(final_poses_path, bag_file, output_dir)
    run_map_quality_analysis(os.path.join(
        output_dir, "map.pcd"), os.path.join(output_dir, "map_quality.json"))
    logger.info("run_map_builder.py finished successfully!")


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])

    if not os.path.exists(args.o):
        logger.error(
            "output path does not exist, exiting. Output directory: %s", args.o)
        exit()

    setup_logger(os.path.join(args.o, "run_map_builder_pipeline.log"))

    run(args.b, args.local_mapper_bag, args.o)
