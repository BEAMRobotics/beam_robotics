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
        f"Loading hyperparameter tuning config json from {config_path}")
    with open(config_path) as f:
        config = json.load(f)
        return config


def get_from_dict(data: Dict, keys_list: List[str]):
    return reduce(operator.getitem, keys_list, data)


def set_in_dict(data: Dict, keys_list: List[str], value: Any):
    get_from_dict(data, keys_list[:-1])[keys_list[-1]] = value

# param descriptions:
# output_path: path where hyperparameter tuning will store results
# validation_json_path: path to json which will have updated results each time we call next() on this class
# config_filename filename for the config file, should be in pipelines/inputs


class HyperParamTuning:
    def __init__(self, output_path: str, validation_json_path: str, config_filename: str = "hyperparameter_tuning.json"):
        self.output_path: str = output_path
        self.hyper_param_tuning_output = os.path.join(
            self.output_path, HYPER_PARAM_TUNING_FOLDER)
        os.makedirs(self.hyper_param_tuning_output, exist_ok=True)
        setup_logger(os.path.join(self.hyper_param_tuning_output,
                     "hyper_parameter_tuning.log"))
        logger.info("------------------------------------------")
        logger.info("Initializing new hyperparameter tuning run")
        logger.info("------------------------------------------")
        config_path = os.path.join(PIPELINE_INPUTS, config_filename)
        self.validation_json_path = validation_json_path
        self.config: Dict = load_json(config_path)
        self.results_path = os.path.join(
            self.hyper_param_tuning_output, PARAM_TUNING_RESULTS_FILENAME)
        self.results = {}
        self.iter: int = 0

        self.__backup_config_files()
        self.rnd_gen = random
        self.rnd_gen.seed()
        self.selected_parameters = None
        self.iteration_failed = False
        self.set_all_parameter_combinations = False

    def next(self) -> bool:
        if self.iter == 0 and self.config["max_iterations"] == 0:
            logger.info(
                "Max iterations set to 0, running once without sampling parameters")
            self.iter += 1
            return True

        if self.selected_parameters is not None:
            self.__store_results()
            self.__output_results()

        if self.iter >= self.config["max_iterations"]:
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
        logger.warn("Setting iteration as failed")
        self.iteration_failed = True

    def store_best_parameter(self, parameter_key: str = "mean_knn_dist_mm", lower_is_better: bool = True):
        if (self.config["max_iterations"] == 0):
            return
        best_iter = self.__get_best_parameter(parameter_key, lower_is_better)
        self.results["best_iter"] = best_iter
        self.__output_results()

    def set_best_parameters(self, parameter_key: str = "mean_knn_dist_mm", lower_is_better: bool = True):
        self.__backup_config_files()
        best_iter = self.__get_best_parameter(parameter_key, lower_is_better)
        logger.info(f"Setting best parameters to iteration {best_iter}")
        self.results["best_iter"] = best_iter
        self.selected_parameters = self.results[best_iter]["parameters"]
        self.__apply_selected_parameters()

    def cleanup(self):
        if (self.config["max_iterations"] == 0):
            return

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

    def get_iteration(self):
        return self.iter

    def get_max_iterations(self):
        return self.config["max_iterations"]

    def __get_best_parameter(self, parameter_key: str = "mean_knn_dist_mm", lower_is_better: bool = True) -> str:
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
            if results["results"] is None:
                logger.warn(
                    f"skipping iter {iter_str} which was not successful")
                continue
            if not results["results"]:
                continue
            value = results["results"][parameter_key]
            if lower_is_better and value < best_value:
                best_value = value
                best_iter = iter_str
            elif not lower_is_better and value > best_value:
                best_value = value
                best_iter = iter_str

        if best_iter == "-1":
            raise Exception("Could not find best value")
        return best_iter

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
        max_iter = self.config["max_iterations"]
        logger.info(
            f"Sampling new set of parameters [{self.iter}/{max_iter}]")
        # iterate through all parameters and sample new values forming a unique combination
        if self.set_all_parameter_combinations:
            self.selected_parameters, is_successful = self.__select_any_parameter_set()
        else:
            self.selected_parameters, is_successful = self.__select_unique_parameter_set()

        if not is_successful:
            if not self.config["allow_duplicate_param_set"]:
                print("exiting parameter tuning")
            else:
                print("continuing with duplicate parameters")
                self.set_all_parameter_combinations = True
                self.selected_parameters, is_successful = self.__select_any_parameter_set()
        return is_successful

    def __select_any_parameter_set(self) -> Tuple[Dict, bool]:
        # {filepath -> {parameter_key -> value}}
        selected_parameters = {}
        for filepath_rel, parameters in self.config["files"].items():
            selected_parameters[filepath_rel] = {}
            for parameter in parameters:
                new_param = self.__sample_parameter(parameter)
                parameter_keys_combined = parameter["parameter"]
                selected_parameters[filepath_rel][parameter_keys_combined] = new_param
        return selected_parameters, True

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
                    new_param = self.__sample_parameter(parameter)
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

    def __sample_parameter(self, parameter_info: Dict) -> Any:

        if "values" in parameter_info:
            num_elements = len(parameter_info["values"])
            id = self.rnd_gen.randint(0, num_elements - 1)
            parameter = parameter_info["values"][id]
            return parameter

        max = parameter_info["max"]
        min = parameter_info["min"]
        increment = parameter_info["increment"]
        num_increments = (max - min) / increment + 1
        increment_id = self.rnd_gen.randint(0, int(num_increments))
        parameter = min + increment_id * increment
        return parameter

    def __store_results(self):
        logger.info(f"Storing results from iteration {self.iter}")
        self.results[str(self.iter)] = {}

        if self.iteration_failed:
            self.results[str(self.iter)
                         ]["parameters"] = self.selected_parameters
            self.results[str(self.iter)]["results"] = {}
            self.iteration_failed = False
            return

        logger.info(
            f"Loading validation file from: {self.validation_json_path}")
        with open(self.validation_json_path, 'r') as f:
            results = json.load(f)
            self.results[str(self.iter)]["results"] = results
        self.results[str(self.iter)]["parameters"] = self.selected_parameters
