import argparse
import os
import sys
import pathlib
from typing import Any, Dict, List
import json

# NOTE: this may differ based on how you installed cloudcompare. You will also need the pcl extensions
CC_BIN = "cloudcompare.CloudCompare"


def parse_args(args) -> Any:
    parser = argparse.ArgumentParser(
        description='Calculate map quality using CloudCompare tools')
    parser.add_argument(
        '-i', type=str, help='path to map file (pcd).')
    parser.add_argument(
        '-o', type=str, help='output path for data. Results filename will be: map_quality.json')
    args = parser.parse_args()
    return args


def load_data(xyz_path: str) -> Dict[str, List[float]]:
    print("reading xyz file...")
    with open(xyz_path, "r") as f:
        lines = [line.strip().split(" ") for line in f.readlines()]

    # setup data and get info
    data = {}
    field_to_col_id = {}
    data_start = 0
    for i in range(len(lines)):
        line = lines[i]
        if line[0] == "FIELDS":
            for k in range(len(line)):
                if line[k] not in ["FIELDS", "x", "y", "z"]:
                    data[line[k]] = []
                    field_to_col_id[line[k]] = k - 1
        elif line[0] == "DATA":
            data_start = i + 1
            break

    # load data
    for i in range(data_start, len(lines)):
        for field, id in field_to_col_id.items():
            d = float(lines[i][id])
            if d != -1:
                data[field].append(d)
    print("Done loading data.")
    return data


def calculate_density(pcd_path: str, output_path: str) -> str:
    print(f"calculating CloudCompare statisitcs for pcd: {pcd_path}")
    map_filename = pathlib.Path(args.i).stem
    save_path = os.path.join(output_path, map_filename + "_cc_density.xyz")
    cmd = f"{CC_BIN} -o {pcd_path} -C_EXPORT_FMT ASC -DENSITY 0.1 -TYPE SURFACE "
    cmd += f"-SAVE_CLOUDS FILE {save_path}"
    print(f"Running command: {cmd}")
    os.system(cmd)
    return save_path


def calculate_roughness(pcd_path: str, output_path: str) -> str:
    print(f"calculating CloudCompare statisitcs for pcd: {pcd_path}")
    map_filename = pathlib.Path(args.i).stem
    save_path = os.path.join(output_path, map_filename + "_cc_roughness.xyz")
    cmd = f"{CC_BIN} -o {pcd_path} -C_EXPORT_FMT ASC -ROUGH 0.1 "
    cmd += f"-SAVE_CLOUDS FILE {save_path}"
    print(f"Running command: {cmd}")
    os.system(cmd)
    return save_path


def get_mean(map_path: str) -> float:
    print(f"reading xyz file: {map_path}")
    with open(map_path, "r") as f:
        lines = [line.strip().split(" ") for line in f.readlines()]

    sum = 0.0
    for line in lines:
        if line[3] == "nan":
            continue
        sum += float(line[3])

    return sum / len(lines)


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])

    if not os.path.exists(args.i):
        raise Exception(f"Invalid input path: {args.i}")

    density_map_path = calculate_density(args.i, args.o)
    roughness_map_path = calculate_roughness(args.i, args.o)
    mean_density = get_mean(density_map_path)
    mean_roughness = get_mean(roughness_map_path)

    results_path = os.path.join(args.o, "map_quality.json")
    results = {}
    results["cc_mean_roughness"] = mean_roughness
    results["cc_mean_density"] = mean_density
    f = open(results_path, "w")
    json.dump(results, f, indent=4)
    f.close()
