import rospkg
import os
from pathlib import Path


def get_pipelines_path() -> str:
    current_file = os.path.abspath(__file__)
    return Path(current_file).parent


def get_pipeline_inputs_path() -> str:
    return os.path.join(get_pipelines_path(), "inputs")


def get_calibration_results_path() -> str:
    calib_publisher_path = rospkg.RosPack().get_path("calibration_publisher")
    calibration_path = Path(calib_publisher_path).parent
    return os.path.join(calibration_path, "results")


# Directories
BEAM_SLAM_LAUNCH_PATH = rospkg.RosPack().get_path("beam_slam_launch")

BS_LAUNCH_FILES_PATH = os.path.join(BEAM_SLAM_LAUNCH_PATH, "launch")
BS_CONFIG_FILES_PATH = os.path.join(BEAM_SLAM_LAUNCH_PATH, "config")

CATKIN_WS = "/userhome/catkin_ws"

PIPELINES_PATH = get_pipelines_path()
PIPELINE_INPUTS = get_pipeline_inputs_path()
BEAM_ROBOTICS_PATH = Path(PIPELINES_PATH).parent


BEAM_CALIBRATION_RESULTS_PATH = get_calibration_results_path()
CALIBRATION_PATH = os.path.join(
    BEAM_CALIBRATION_RESULTS_PATH, "inspector_gadget2/current")
# BS_CALIB_FILES_PATH = os.path.join(BEAM_SLAM_LAUNCH_PATH, "calibrations")
# EXTRINSICS_PATH = os.path.join(CALIBRATION_PATH, "extrinsics")
# EXTRINSICS_PATH = os.path.join(BS_CALIB_FILES_PATH, "ig2")

EXTRINSICS_PATH = os.path.join(PIPELINES_PATH, "calibrations")
EXTRINSICS_JSON_PATH = os.path.join(
    EXTRINSICS_PATH, "extrinsics.json")
INTRINSICS_PATH = os.path.join(
    CALIBRATION_PATH, "intrinsics")

# Folder names
RESULTS_FOLDER = "results"
SLAM_OUTPUT_FOLDER = "slam"
MAP_BUILDER_FOLDER = "map_builder"
GLOBAL_MAPPER_RESULTS = "global_mapper_results"
GLOBAL_MAP_REFINEMENT_RESULTS = "global_map_refined_results"
IMAGE_EXTRACTOR_FOLDER = "image_extractor"
MAP_LABELER_FOLDER = "map_labeler"
HYPER_PARAM_TUNING_FOLDER = "hyper_param_tuning"

# Filenames
SLAM_BAG_FILE = "data.bag"
INSPECTION_BAG_FILE = "inspection.bag"
LOCAL_MAPPER_BAG_FILE = "local_mapper_results.bag"
MAP_QUALITY_FILENAME = "map_quality.json"
