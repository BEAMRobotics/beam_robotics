import rospkg
import os
from pathlib import Path


def get_pipelines_path() -> str:
    current_file = os.path.abspath(__file__)
    return Path(current_file).parent


def get_pipeline_inputs_path() -> str:
    return os.path.join(get_pipelines_path(), "inputs")


# Directories
BEAM_SLAM_LAUNCH_PATH = rospkg.RosPack().get_path("beam_slam_launch")
BS_LAUNCH_FILES_PATH = os.path.join(BEAM_SLAM_LAUNCH_PATH, "launch")
BS_CONFIG_FILES_PATH = os.path.join(BEAM_SLAM_LAUNCH_PATH, "config")
BS_CALIB_FILES_PATH = os.path.join(BEAM_SLAM_LAUNCH_PATH, "calibrations")
CATKIN_WS = "/userhome/catkin_ws"

PIPELINES_PATH = get_pipelines_path()
PIPELINE_INPUTS = get_pipeline_inputs_path()

# Folder names
RESULTS_FOLDER = "results"
SLAM_OUTPUT_FOLDER = "slam"
MAP_BUILDER_FOLDER = "map_builder"
GLOBAL_MAPPER_RESULTS = "global_map_results"
GLOBAL_MAP_REFINEMENT_RESULTS = "global_map_refined_results"

# Filenames
RAW_BAG_FILE = "data.bag"
LOCAL_MAPPER_BAG_FILE = "local_mapper_results.bag"

# File paths
EXTRINSICS_PATH = os.path.join(BS_CALIB_FILES_PATH, "ig2/extrinsics.json")
