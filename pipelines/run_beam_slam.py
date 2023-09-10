import argparse
import sys
import os
from typing import Any
import roslaunch
import rospy
import rospkg
import logging

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
beam_slam_launch_path = rospkg.RosPack().get_path("beam_slam_launch")
launch_files_path = os.path.join(beam_slam_launch_path, "launch")
config_files_path = os.path.join(beam_slam_launch_path, "config")
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
    parser.add_argument('-e', help='end time in s', type=float)
    parser.add_argument('-r', help='rosbag play rate', type=float, default=1)
    parser.add_argument(
        '-o', help='full path to output directory', type=str, default="")
    parser.add_argument(
        '-slam_config', help='filename of slam config yaml file', type=str, default="lio.yaml")
    args = parser.parse_args()
    return args


def start_ros_master() -> Any:
    logger.info("starting ROS master")
    rosmaster = roslaunch.parent.ROSLaunchParent(
        uuid, roslaunch_files=[], is_core=True)
    rosmaster.start()
    return rosmaster


def start_calibration_publisher():
    launch_file_path = os.path.join(
        launch_files_path, "calibration_publisher.launch")
    logger.info("running launch file: %s", launch_file_path)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
    launch.start()


def load_calibration_params():
    config_file_path = os.path.join(
        config_files_path, "calibration_params.yaml")
    logger.info("loading calibration params from: %s", config_file_path)
    os.system("rosparam load " + config_file_path)


def load_slam_params(slam_yaml_filename):
    config_file_path = os.path.join(config_files_path, slam_yaml_filename)
    logger.info("loading SLAM params from: %s", config_file_path)
    os.system("rosparam load " + config_file_path + " /local_mapper")


def run_slam():
    logger.info("starting SLAM node")
    local_mapper_node = roslaunch.core.Node(
        package='fuse_optimizers', node_type='fixed_lag_smoother_node',
        name='local_mapper')
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(local_mapper_node)
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
    topics = "/local_mapper/path_pub/pose_array "
    topics += "/local_mapper/path_pub/path "
    topics += "/local_mapper/inertial_odometry/odometry "
    topics += "/local_mapper/lidar_odometry/marginalized "
    topics += "/local_mapper/lidar_odometry/odometry "
    topics += "/local_mapper/path_publisher/path "
    topics += "/local_mapper/path_publisher/pose_array "
    topics += "/local_mapper/visual_odometry/odometry "
    topics += "/local_mapper/visual_odometry/pose "
    
    save_path = os.path.join(output_dir, "slam_results.bag")
    rosbag_args = '-O {} {}'.format(save_path, topics)
    logger.info("running: rosbag record {}".format(rosbag_args))
    rosbag_node = roslaunch.core.Node(
        package='rosbag', node_type='record',
        name='rosbag_record', args="{}".format(rosbag_args))

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(rosbag_node)
    return process


def run(bag_file: str, start_time: float, end_time: float, rate: float,
        slam_params_filename: str, output_dir: str):
    rosmaster = start_ros_master()
    start_calibration_publisher()
    rospy.init_node('run_beam_slam_pipeline')
    load_calibration_params()
    load_slam_params(slam_params_filename)
    rospy.sleep(2) # sleep to give the computer time to start everything
    slam_node_process = run_slam()
    rospy.sleep(2) # sleep to give the computer time to start everything
    rosbag_play_node_process = run_rosbag_play(bag_file, start_time, rate)
    rosbag_record_node_process = run_rosbag_record(output_dir)

    start_time = rospy.get_rostime()
    end_time_at_run_rate = end_time / rate
    while ((rospy.get_rostime() - start_time).to_sec() < end_time_at_run_rate):
        if not slam_node_process.is_alive():
            logger.error("SLAM node shutdown, exiting")
            break
        if not rosbag_play_node_process.is_alive():
            logger.error("rosbag play shutdown, exiting")
            break
        if not rosbag_record_node_process.is_alive():
            logger.error("rosbag record shutdown, exiting")
            break

    time_elapsed = (rospy.get_rostime() - start_time)
    if time_elapsed.to_sec() >= end_time_at_run_rate:
        logger.warning(
            "time elapsed exceeded user input (%s), exiting", end_time)

    logger.info("total runtime: %s s", time_elapsed.secs * rate)


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])

    if not args.o:
        setup_logger(None)
        logger.warning("no output directory specified, not saving results")
    elif not os.path.exists(args.o):
        logger.error(
            "output path does not exist, exiting. Output directory: %s", args.o)
    else:
        setup_logger(os.path.join(args.o, "run_beam_slam_pipeline.log"))

    run(args.b, args.s, args.e, args.r, args.slam_config, args.o)
