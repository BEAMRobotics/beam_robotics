import argparse
import sys
import os
from typing import Any
import roslaunch
import rospy
import rospkg
import logging

from utils import start_ros_master, start_calibration_publisher
from params import *

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
logger = logging.getLogger("RUN_BEAM_SLAM")


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
    parser.add_argument('-b', help='input bag file')
    parser.add_argument('-s', help='start time in s', type=float, default=1)
    parser.add_argument(
        '-d', help='duration to play in s. Set to -1 to play till end of bag', type=float)
    parser.add_argument('-r', help='rosbag play rate', type=float, default=1)
    parser.add_argument(
        '-o', help='full path to output directory', type=str, default="")
    parser.add_argument(
        '-local_mapper_config', help='filename of local mapper config yaml file', type=str, default="lio.yaml")
    parser.add_argument(
        '-global_mapper_config', help='filename of local mapper config yaml file', type=str, default="global_mapper.yaml")
    args = parser.parse_args()
    return args


def load_calibration_params():
    config_file_path = os.path.join(
        BS_CONFIG_FILES_PATH, "calibration_params.yaml")
    logger.info("loading calibration params from: %s", config_file_path)
    os.system("rosparam load " + config_file_path)


def load_slam_params(lm_filename: str, gm_filename: str, output_dir: str):
    config_file_path = os.path.join(BS_CONFIG_FILES_PATH, lm_filename)
    logger.info("loading SLAM local mapper params from: %s", config_file_path)
    os.system("rosparam load " + config_file_path + " /local_mapper")

    config_file_path = os.path.join(BS_CONFIG_FILES_PATH, gm_filename)
    logger.info("loading SLAM global mapper params from: %s", config_file_path)
    os.system("rosparam load " + config_file_path + " /global_mapper")

    # override output path
    os.system("rosparam set /global_mapper/global_mapper/output_path " + output_dir)


def run_local_mapper():
    logger.info("starting local mapper node")
    local_mapper_node = roslaunch.core.Node(
        package='bs_optimizers', node_type='fixed_lag_smoother_node',
        name='local_mapper',
        output='screen')
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(local_mapper_node)
    return process


def run_global_mapper():
    logger.info("starting global mapper node")
    global_mapper_node = roslaunch.core.Node(
        package='bs_optimizers', node_type='fixed_lag_smoother_node',
        name='global_mapper',
        output='screen')
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(global_mapper_node)
    return process


def run_rosbag_play(bag_file: str, start_time: float, rate: float):
    rosbag_args = '-s {} -r {} {}'.format(start_time, rate, bag_file)
    logger.info("running: rosbag play {}".format(rosbag_args))
    rosbag_node = roslaunch.core.Node(
        package='rosbag', node_type='play',
        name='rosbag_play', args="{}".format(rosbag_args))

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(rosbag_node)
    return process


def run_rosbag_record(output_dir: str):
    topics = "/local_mapper/graph_publisher/poses "
    topics += "/local_mapper/graph_publisher/odom "
    topics += "/local_mapper/inertial_odometry/odometry "
    topics += "/local_mapper/lidar_odometry/odometry "
    topics += "/local_mapper/visual_odometry/odometry "

    save_path = os.path.join(output_dir, LOCAL_MAPPER_BAG_FILE)
    rosbag_args = '-O {} {}'.format(save_path, topics)
    logger.info("running: rosbag record {}".format(rosbag_args))
    rosbag_node = roslaunch.core.Node(
        package='rosbag', node_type='record',
        name='rosbag_record', args="{}".format(rosbag_args))

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(rosbag_node)
    return process


def run(bag_file: str, start_time: float, duration: float, rate: float,
        local_mapper_params_filename: str, global_mapper_params_filename: str, output_dir: str):
    rosmaster = start_ros_master()
    start_calibration_publisher()
    rospy.init_node('run_beam_slam_pipeline')
    load_calibration_params()
    load_slam_params(local_mapper_params_filename,
                     global_mapper_params_filename,
                     output_dir)
    rospy.sleep(2)  # sleep to give the computer time to start everything
    slam_lm_node_process = run_local_mapper()
    rospy.sleep(2)  # sleep to give the computer time to start everything
    slam_gm_node_process = run_global_mapper()
    rospy.sleep(2)  # sleep to give the computer time to start everything
    rosbag_play_node_process = run_rosbag_play(bag_file, start_time, rate)
    rosbag_record_node_process = run_rosbag_record(output_dir)

    bag_start_time = rospy.get_rostime().to_sec()
    end_time_at_run_rate = bag_start_time + duration / rate
    while (True):
        if duration != -1:
            if rospy.get_rostime().to_sec() > end_time_at_run_rate:
                logger.info(
                    "time elapsed exceeded user input (%s), exiting", duration)
                break
        if not slam_lm_node_process.is_alive():
            logger.error("SLAM local mapper node shutdown, exiting")
            break
        if not slam_gm_node_process.is_alive():
            logger.error("SLAM global mapper node shutdown, exiting")
            break
        if not rosbag_play_node_process.is_alive():
            logger.error("rosbag play shutdown, exiting")
            break
        if not rosbag_record_node_process.is_alive():
            logger.error("rosbag record shutdown, exiting")
            break
    
    slam_lm_node_process.stop()
    slam_gm_node_process.stop()
    rosbag_play_node_process.stop()
    rosbag_record_node_process.stop()
    logger.info("run_beam_slam.py finished successfully!")


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])

    if not args.o:
        setup_logger(None)
        logger.warning("no output directory specified, not saving results")
    elif not os.path.exists(args.o):
        logger.error(
            "output path does not exist, exiting. Output directory: %s", args.o)
        exit()
    else:
        setup_logger(os.path.join(args.o, "run_beam_slam_pipeline.log"))

    run(args.b, args.s, args.d, args.r, args.local_mapper_config,
        args.global_mapper_config, args.o)
