import argparse
import sys
import os
import json
import logging
from typing import List
import numpy as np
import matplotlib.pyplot as plt

logger = logging.getLogger("PLOTTING")

def setup_logger():
    logger.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        '[%(levelname)s] %(asctime)s-%(name)s: %(message)s')
    handler1 = logging.StreamHandler(sys.stdout)
    handler1.setLevel(logging.DEBUG)
    handler1.setFormatter(formatter)
    logger.addHandler(handler1)


def parse_args(args):
    parser = argparse.ArgumentParser(
        description='Plot the results from hyper parameter tuning')
    parser.add_argument('-r', type=str, help='results filepath (json)')
    parser.add_argument('-o', type=str, help='outpath folder path')
    parser.add_argument('--metrics', nargs='+', type=str, help='results metric name(s))')
    args = parser.parse_args()
    return args


def main(results_path: str, output_path: str, metrics: List[str]):
    if not os.path.exists(results_path):
        raise Exception(f"input results file not found: {results_path}")
    results = {}
    with open(results_path, 'r') as f:
        results = json.load(f)
    for metric in metrics:
        # param name to list of results for this metric
        param_name_to_metrics = {} # results with this param value
        param_name_to_values = {} # param values
        for iter, iter_results in results.items():
            if iter == "best_iter":
                continue
            if "results" not in iter_results:
                raise Exception("missing key 'results' in json")
            if "parameters" not in iter_results:
                raise Exception("missing key 'parameters' in json")            
            
            if not iter_results["results"]:
                logger.info(f"results empty for iter {iter}, skipping")
                continue
            if metric not in iter_results["results"]:
                raise Exception(f"metric {metric} missing  in json")            
            metric_value = iter_results["results"][metric]
            for file, params in iter_results["parameters"].items():
                for param_name, param_value in params.items():
                    if param_name not in param_name_to_metrics:
                        param_name_to_metrics[param_name] = []
                        param_name_to_values[param_name] = []
                    param_name_to_metrics[param_name].append(metric_value)
                    param_name_to_values[param_name].append(param_value)
        
        for param_name, metric_value in param_name_to_metrics.items():
            filename = f"{metric}_results_for_param_{param_name}.png"
            savepath = os.path.join(output_path, filename)
            
            x = param_name_to_values[param_name]
            y = param_name_to_metrics[param_name]

            fig = plt.figure()
            plt.figure().clear()
            plt.close()
            plt.cla()
            plt.clf()

            plt.scatter(np.array(x), np.array(y))
            plt.title('Hyper Param Tuning Results')
            plt.xlabel(f"{param_name} values")
            plt.ylabel(f"{metric} results")

            # draw line with means for each value
            unique_values_to_results = {}
            for i in range(len(x)):
                if x[i] not in unique_values_to_results:
                    unique_values_to_results[x[i]] = []
                unique_values_to_results[x[i]].append(y[i]) 
            xx = []
            yy = []
            for v in sorted(unique_values_to_results.keys()):
                mean = np.mean(np.array(unique_values_to_results[v]))
                xx.append(v)
                yy.append(mean)

            # for v, rs in unique_values_to_results.items():
                # mean = np.mean(np.array(rs))
                # xx.append(v)
                # yy.append(mean)
            plt.plot(xx, yy)


            logger.info(f"saving plot to: {savepath}")
            plt.savefig(savepath)




if __name__ == "__main__":
    setup_logger()
    args = parse_args(sys.argv[1:])
    main(args.r, args.o, args.metrics)
