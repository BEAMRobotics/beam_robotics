import sys
import os
import shutil
from typing import Any, Dict, Tuple, List
import logging
import json
import random
from functools import reduce
import operator

from params import *

logger = logging.getLogger("HYPERPARAMETER_TUNING")

CONFIG_FILE_BACKUP_POST_FIX = ".backup.json"
RESULTS_FILENAME = "hyperparameter_tuning_results.json"


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


def load_json(config_path: str):
    logger.info(
        f"loading hyperparameter tuning config json from {config_path}")
    with open(config_path) as f:
        config = json.load(f)
        return config


def get_from_dict(data: Dict, keys_list: List[str]):
    return reduce(operator.getitem, keys_list, data)


def set_in_dict(data: Dict, keys_list: List[str], value: Any):
    get_from_dict(data, keys_list[:-1])[keys_list[-1]] = value


class HyperParamTuning:
    def __init__(self, output_path: str, continue_from_previous: bool = True):
        self.output_path: str = output_path
        setup_logger(os.path.join(self.output_path,
                     "hyper_parameter_tuning.log"))
        config_path = os.path.join(
            PIPELINE_INPUTS, "hyperparameter_tuning.json")
        self.config: Dict = load_json(config_path)
        self.results_path = os.path.join(self.output_path, RESULTS_FILENAME)
        if continue_from_previous and os.path.exists(self.results_path):
            self.__load_results()
        else:
            self.iter: int = 0
            self.results = {}
            self.max_iterations: int = self.config["max_iterations"]

        self.__backup_config_files()
        self.rnd_gen = random
        self.rnd_gen.seed()
        self.selected_parameters = None
        self.iteration_failed = False

    def next(self) -> bool:
        if self.selected_parameters is not None:
            self.__store_results()

        if self.iter >= self.max_iterations:
            logger.info(f"Hit max number of iterations: {self.iter}")
            self.cleanup()
            return False

        self.iter += 1
        is_successful = self.__sample_parameters()
        if not is_successful:
            self.cleanup()
            return False
        self.__apply_selected_parameters()
        return True

    def mark_as_failed(self):
        self.iteration_failed = True

    def set_best_parameters(self, parameter_key: str = "mean_knn_dist_mm", lower_is_better: bool = True):
        self.__backup_config_files()

        if lower_is_better:
            search_for = "lowest"
            best_value = 9e9
        else:
            search_for = "highest"
            best_value = -9e9
        logger.info(
            f"Setting parameters to the best results by looking for the {search_for} {parameter_key}")

        best_iter = "-1"
        for iter_str, results in self.results.items():
            if results["map_quality"] is None:
                logger.warn(
                    f"skipping iter {iter_str} which was not successful")
                continue

            value = results["map_quality"][parameter_key]
            if lower_is_better and value < best_value:
                best_value = value
                best_iter = iter_str
            elif not lower_is_better and value > best_value:
                best_value = value
                best_iter = iter_str

        if best_iter == "-1":
            raise Exception("Could not find best value")

        logger.info(f"Setting best parameters to iteration {best_iter}")
        self.selected_parameters = self.results[best_iter]["parameters"]
        self.__apply_selected_parameters()

    def cleanup(self):
        logger.info("Cleaning up temporary config files")
        for filepath_rel, _ in self.config["files"].items():
            filepath = os.path.join(BEAM_ROBOTICS_PATH, filepath_rel)
            if not os.path.exists(filepath):
                raise Exception(f"Invalid filepath: {filepath_rel}")
            backup_path = filepath + CONFIG_FILE_BACKUP_POST_FIX
            if not os.path.exists(backup_path):
                return
            if os.path.exists(filepath):
                os.remove(filepath)
            shutil.copy(backup_path, filepath)
            os.remove(backup_path)
        self.__output_results()

    def __load_results(self):
        logger.info(f"Loading results from: {self.results_path }")
        with open(self.results_path, 'r') as f:
            self.results = json.load(f)

        # get max number of iterations
        previous_max = 0
        for iter_str, _ in self.results.items():
            if int(iter_str) > previous_max:
                previous_max = int(iter_str)
        self.iter: int = previous_max
        self.max_iterations: int = self.iter + self.config["max_iterations"]
        logger.info(
            f"set current iteration to {self.iter} and max iterations to {self.max_iterations}")

    def __output_results(self):
        logger.info(f"Outputting results to: {self.results_path}")
        with open(self.results_path, 'w') as f:
            json.dump(self.results, f, indent=4)

    def __backup_config_files(self):
        logger.info("Backing up original config files")
        for filepath_rel, _ in self.config["files"].items():
            filepath = os.path.join(BEAM_ROBOTICS_PATH, filepath_rel)
            if not os.path.exists(filepath):
                raise Exception(f"Invalid filepath: {filepath_rel}")
            backup_path = filepath + CONFIG_FILE_BACKUP_POST_FIX
            if os.path.exists(backup_path):
                os.remove(backup_path)
            shutil.copy(filepath, backup_path)

    def __apply_selected_parameters(self):
        logger.info("Applying selected parameters to config files")
        for filepath_rel, parameters in self.selected_parameters.items():
            filepath = os.path.join(BEAM_ROBOTICS_PATH, filepath_rel)
            backup_path = filepath + CONFIG_FILE_BACKUP_POST_FIX
            logger.info(f"updating config file: {filepath}")
            config_data = {}
            with open(backup_path, "r") as f:
                config_data = json.load(f)
            for keys_list_combined, value in parameters.items():
                keys_list = keys_list_combined.split('/')
                set_in_dict(config_data, keys_list, value)

            with open(filepath, 'w') as f:
                json.dump(config_data, f, indent=4)

    def __sample_parameters(self) -> bool:
        logger.info(
            f"Sampling new set of parameters [{self.iter}/{self.max_iterations}]")
        # iterate through all parameters and sample new values forming a unique combination
        self.selected_parameters, is_successful = self.__select_unique_parameter_set()
        if not is_successful:
            return False
        return True

    def __select_unique_parameter_set(self) -> Tuple[Dict, bool]:
        # {filepath -> {parameter_key -> value}}
        selected_parameters = {}
        # iterate until we find one combination that is unique
        MAX_SEARCH_ATTEMPTS = 10000
        for i in range(MAX_SEARCH_ATTEMPTS):
            selected_parameters = {}
            for filepath_rel, parameters in self.config["files"].items():
                selected_parameters[filepath_rel] = {}
                for parameter in parameters:
                    new_param = self.__sample_parameter(
                        min=parameter["min"],
                        max=parameter["max"],
                        increment=parameter["increment"],
                    )
                    parameter_keys_combined = parameter["parameter"]
                    selected_parameters[filepath_rel][parameter_keys_combined] = new_param

            if not self.__has_parameter_set_been_used(selected_parameters):
                return selected_parameters, True
        logger.warning(
            "Hit max number of search attempts when looking for a new unique parameter set")
        return {}, False

    def __has_parameter_set_been_used(self, parameter_set: Dict) -> bool:
        if not self.results:
            return False

        for iter, results in self.results.items():
            if parameter_set == results["parameters"]:
                return True

        return False

    def __sample_parameter(self, min: Any, max: Any, increment: Any) -> Any:
        num_increments = (max - min) / increment
        increment_id = self.rnd_gen.randint(0, num_increments)
        parameter = min + increment_id * increment
        return parameter

    def __store_results(self):
        logger.info(f"Storing results from iteration {self.iter}")
        self.results[str(self.iter)] = {}

        if self.iteration_failed:
            self.results[str(self.iter)
                         ]["parameters"] = self.selected_parameters
            self.results[str(self.iter)]["map_quality"] = {}
            self.iteration_failed = False

        map_quality_results_path = os.path.join(
            self.output_path, MAP_BUILDER_FOLDER, MAP_QUALITY_FILENAME)
        with open(map_quality_results_path, 'r') as f:
            results = json.load(f)
            self.results[str(self.iter)]["map_quality"] = results
        self.results[str(self.iter)]["parameters"] = self.selected_parameters
