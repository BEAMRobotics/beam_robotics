import argparse
import sys
import os
from typing import Any
import roslaunch
import rospy
import rospkg

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
beam_slam_launch_path = rospkg.RosPack().get_path("beam_slam_launch")
launch_files_path = os.path.join(beam_slam_launch_path, "launch")
config_files_path = os.path.join(beam_slam_launch_path, "config")

def parse_args(args) -> Any:
    parser = argparse.ArgumentParser(description='Run SLAM')
    parser.add_argument('--bag_file', nargs=1, help='input bag file')
    args = parser.parse_args()
    return args

def start_ros_master() -> Any:    
    rosmaster = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files=[], is_core=True)
    rosmaster.start()
    return rosmaster

def start_calibration_publisher():
    launch_file_path = os.path.join(launch_files_path, "calibration_publisher.launch")
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
    launch.start()

def load_calibration_params():
    config_file_path = os.path.join(config_files_path, "calibration_params.yaml")
    os.system("rosparam load " + config_file_path)

def load_slam_params(slam_yaml_filename):
    config_file_path = os.path.join(config_files_path,slam_yaml_filename)
    os.system("rosparam load " + config_file_path + " /local_mapper")    

def run_slam(bag_file: str):
    # rosmaster = start_ros_master() # only needed if no launch file is used
    start_calibration_publisher()
    load_calibration_params()
    load_slam_params("lio.yaml")

    local_mapper_node = roslaunch.core.Node(
        package='fuse_optimizers', node_type='fixed_lag_smoother_node', 
        name='local_mapper')
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(local_mapper_node)

    counter = 0
    while(counter < 30):
       print("sleeping...")
       rospy.sleep(1)
       counter = counter +1


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])
    run_slam(args.bag_file)
