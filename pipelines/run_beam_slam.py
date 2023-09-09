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

def setup_logger():
    logger.setLevel(logging.DEBUG)
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter('[%(levelname)s] %(asctime)s-%(name)s: %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)

def parse_args(args) -> Any:
    parser = argparse.ArgumentParser(description='Run SLAM')
    parser.add_argument('-b', help='input bag file')
    parser.add_argument('-s', help='start time in s', type=float, default=1)
    parser.add_argument('-e', help='end time in s', type=float)
    parser.add_argument('-r', help='rosbag play rate', type=float, default=1)
    args = parser.parse_args()
    return args

def start_ros_master() -> Any:  
    logger.info("starting ROS master")  
    rosmaster = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files=[], is_core=True)
    rosmaster.start()
    return rosmaster

def start_calibration_publisher():
    launch_file_path = os.path.join(launch_files_path, "calibration_publisher.launch")
    logger.info("running launch file: %s", launch_file_path)  
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
    launch.start()

def load_calibration_params():
    config_file_path = os.path.join(config_files_path, "calibration_params.yaml")
    logger.info("loading calibration params from: %s", config_file_path)  
    os.system("rosparam load " + config_file_path)

def load_slam_params(slam_yaml_filename):
    config_file_path = os.path.join(config_files_path,slam_yaml_filename)
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

def run_rosbag(bag_file: str, start_time: float, rate: float):
    rosbag_args = '-s {} -r {} {}'.format(start_time, rate, bag_file)
    logger.info("running: rosbag play {}".format(rosbag_args))  
    rosbag_node = roslaunch.core.Node(
        package='rosbag', node_type='play', 
        name='rosbag_play', args="{}".format(rosbag_args))
    
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(rosbag_node)
    return process

def run(bag_file: str, start_time: float, end_time: float, rate: float):
    rospy.init_node('run_beam_slam_pipeline')
    # rosmaster = start_ros_master() # only needed if no launch file is used
    start_calibration_publisher()
    load_calibration_params()
    load_slam_params("lio.yaml")
    slam_node_process = run_slam()
    rosbag_node_process = run_rosbag(bag_file, start_time, rate)
    

    start_time = rospy.get_rostime()
    end_time_at_run_rate = end_time / rate
    while((rospy.get_rostime() -  start_time).to_sec() < end_time_at_run_rate):
       if not slam_node_process.is_alive():
           logger.error("SLAM node shutdown, exiting")
           break
       if not rosbag_node_process.is_alive():
           logger.error("rosbag play shutdown, exiting")
           break
    
    time_elapsed = (rospy.get_rostime() -  start_time)
    if  time_elapsed.to_sec() >= end_time_at_run_rate:
        logger.warning("time elapsed exceeded user input (%s), exiting", end_time)

    logger.info("total runtime: %s s", time_elapsed.sec * rate)

if __name__ == "__main__":
    setup_logger()
    args = parse_args(sys.argv[1:])
    run(args.b, args.s, args.e, args.r)
