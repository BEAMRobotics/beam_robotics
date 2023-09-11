import argparse
import sys
import os
from typing import Any
import roslaunch
import rospy
import rospkg
import logging
from pathlib import Path

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
logger = logging.getLogger("RUN_MAP_BUILDER")
BIN_PATH = "/userhome/catkin_ws/devel/lib/map_builder"

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
    parser.add_argument('-b', type=str, help='input bag file which contains the lidar data')
    parser.add_argument('-slam_output', type=str,
                        help='path to slam output directory. This should contain slam_results.bag with odometry topics')
    parser.add_argument('-o', type=str, help='full path to output directory')
    args = parser.parse_args()
    return args

def export_raw_slam_poses(type: str, topic: str, bag_file: str, output_dir: str, prefix: str):
    bag_to_poses_bin = os.path.join(BIN_PATH, "map_builder_bag_to_poses_file")
    cmd = "{} -bag {} -output_path {} -topic {} -output_type {}".format(bag_to_poses_bin, bag_file, output_dir,  topic, type)
    logger.info("running command: %s", cmd)
    os.system(cmd)
    if type == "PCD":
        p1 = os.path.join(output_dir, "poses.pcd")
        if not os.path.exists(p1):
            logger.warning("no poses file generated for %s", prefix)
            return
        p2 = os.path.join(output_dir, prefix + "_poses.pcd")
        logger.info("renaming %s to %s", p1, p2)
        os.system("mv " + p1 + " " + p2)
    elif type == "JSON":
        p1 = os.path.join(output_dir, "poses.json")
        if not os.path.exists(p1):
            logger.warning("no poses file generated for %s", prefix)
            return
        p2 = os.path.join(output_dir, prefix + "_poses.json")
        logger.info("renaming %s to %s", p1, p2)
        os.system("mv " + p1 + " " + p2)       

def export_corrected_poses(type: str, poses_low_rate: str, poses_high_rate: str, output_dir: str, prefix: str):
    bag_to_poses_bin = os.path.join(BIN_PATH, "map_builder_fill_in_trajectory")
    cmd = "{} -output_path {} -poses_high_rate {} -poses_low_rate {} -output_type {}".format(bag_to_poses_bin, output_dir,  poses_high_rate, poses_low_rate, type)
    logger.info("running command: %s", cmd)
    os.system(cmd)
    if type == "PCD":
        p1 = os.path.join(output_dir, "poses.pcd")
        if not os.path.exists(p1):
            logger.warning("no poses file generated for %s", prefix)
            return
        p2 = os.path.join(output_dir, prefix + "_poses.pcd")
        logger.info("renaming %s to %s", p1, p2)
        os.system("mv " + p1 + " " + p2)
    elif type == "JSON":
        p1 = os.path.join(output_dir, "poses.json")
        if not os.path.exists(p1):
            logger.warning("no poses file generated for %s", prefix)
            return
        p2 = os.path.join(output_dir, prefix + "_poses.json")
        logger.info("renaming %s to %s", p1, p2)
        os.system("mv " + p1 + " " + p2)               

def build_map(poses_path: str, bag_file: str, output_dir: str):
    build_map_bin = os.path.join(BIN_PATH, "map_builder_build_map")
    beam_slam_launch_path = rospkg.RosPack().get_path("beam_slam_launch")
    calibrations_path = os.path.join(beam_slam_launch_path, "calibrations")
    extrinsics_path = os.path.join(calibrations_path, "ig2/extrinsics.json")
    current_file = os.path.abspath(__file__)
    current_path = Path(current_file).parent
    inputs_dir = os.path.join(current_path, "inputs")
    config_path = os.path.join(inputs_dir, "map_builder_config.json")
    cmd = "{} --bag_file {} --config_file {} --extrinsics {} --output_directory {} --pose_file {}".format(build_map_bin, bag_file, config_path, extrinsics_path, output_dir, poses_path)
    logger.info("Running command: %s", cmd)
    os.system(cmd)

def run(bag_file: str, slam_output_dir: str, output_dir: str):
    slam_bag_file = os.path.join(slam_output_dir, "slam_results.bag")
    export_raw_slam_poses("JSON", "/local_mapper/path_publisher/path", slam_bag_file, output_dir, "local_mapper")
    export_raw_slam_poses("PCD", "/local_mapper/path_publisher/path", slam_bag_file, output_dir, "local_mapper")
    export_raw_slam_poses("JSON", "/local_mapper/inertial_odometry/odometry", slam_bag_file, output_dir, "inertial_odometry")
    export_raw_slam_poses("PCD", "/local_mapper/inertial_odometry/odometry", slam_bag_file, output_dir, "inertial_odometry")
    export_raw_slam_poses("JSON", "/local_mapper/lidar_odometry/odometry", slam_bag_file, output_dir, "lidar_odometry")
    export_raw_slam_poses("PCD", "/local_mapper/lidar_odometry/odometry", slam_bag_file, output_dir, "lidar_odometry")
    export_raw_slam_poses("JSON", "/local_mapper/visual_odometry/odometry", slam_bag_file, output_dir, "visual_odometry")
    export_raw_slam_poses("PCD", "/local_mapper/visual_odometry/odometry", slam_bag_file, output_dir, "visual_odometry")

    poses_high_rate = os.path.join(output_dir, "inertial_odometry_poses.json")
    poses_low_rate = os.path.join(output_dir, "local_mapper_poses.json")
    export_corrected_poses("PCD", poses_low_rate, poses_high_rate, output_dir, "final")
    export_corrected_poses("JSON", poses_low_rate, poses_high_rate, output_dir, "final")

    final_poses_path = os.path.join(output_dir, "final_poses.json")
    build_map(final_poses_path, bag_file, output_dir)

if __name__ == "__main__":
    args = parse_args(sys.argv[1:])

    if not os.path.exists(args.o):
        logger.error(
            "output path does not exist, exiting. Output directory: %s", args.o)
        exit()
    
    setup_logger(os.path.join(args.o, "run_map_builder_pipeline.log"))

    run(args.b, args.slam_output, args.o)
