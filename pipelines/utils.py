import sys
import os
from typing import Any
import roslaunch
import logging

from params import *

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
logger_utils = logging.getLogger("UTILS")


def setup_logger():
    logger_utils.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        '[%(levelname)s] %(asctime)s-%(name)s: %(message)s')
    handler1 = logging.StreamHandler(sys.stdout)
    handler1.setLevel(logging.DEBUG)
    handler1.setFormatter(formatter)
    logger_utils.addHandler(handler1)


def start_ros_master() -> Any:
    setup_logger()
    logger_utils.info("starting ROS master")
    rosmaster = roslaunch.parent.ROSLaunchParent(
        uuid, roslaunch_files=[], is_core=True)
    rosmaster.start()
    return rosmaster


def start_calibration_publisher():
    setup_logger()
    launch_file_path = os.path.join(
        BS_LAUNCH_FILES_PATH, "calibration_publisher.launch")
    logger_utils.info("running launch file: %s", launch_file_path)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
    launch.start()


def start_calibration_publisher_with_file(filepath):
    setup_logger()
    logger_utils.info(
        f"starting calibration publisher using filepath: {filepath}")
    os.system(
        "rosparam set /calibration_publisher_main/extrinsics_file_path " + filepath)
    # os.system(
    #     "rosparam set /calibration_publisher_main/robot_name " + "")
    publisher_node = roslaunch.core.Node(
        package='calibration_publisher', node_type='calibration_publisher_main',
        name='calibration_publisher_main',
        output='screen')
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(publisher_node)
    return process
