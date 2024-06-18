import argparse
import os
import sys
import pathlib
from typing import Any, Dict, List
import matplotlib.pyplot as plt

PCD_TO_XYZ_BIN = "pcl_convert_pcd_ascii_binary"


def parse_args(args) -> Any:
    parser = argparse.ArgumentParser(
        description='Plot the statistic of a map quality pointcloud file. We first use CloudCompare to convert from pcd to .asc')
    parser.add_argument(
        '-i', type=str, help='path to map file. If .pcd, we will use pcl binary to convert to xyz, if already xyz then we directly read results.')
    parser.add_argument('-o', type=str, help='output path')
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


def plot_statistics(xyz_path: str, output_path: str):
    print(f"plotting statistics for {xyz_path}")
    data = load_data(xyz_path)
    for field, field_data in data.items():
        # crop = 100
        crop = len(field_data)
        ids = list(range(0, crop))
        data_croped = field_data[:crop]
        plt.scatter(ids, data_croped)
        plt.title(field)
        img_path = os.path.join(output_path, f"map_quality_{field}_raw.png")
        print(f"saving fig: {img_path}")
        # plt.show()
        plt.savefig(img_path)
        plt.close()

        plt.hist(data_croped, bins=20)
        plt.title(field + " histogram")
        img_path = os.path.join(output_path, f"map_quality_{field}_hist.png")
        print(f"saving fig: {img_path}")
        # plt.show()
        plt.savefig(img_path)
        plt.close()


def pcd_to_xyz(pcd_path: str, output_path: str) -> str:
    print(f"creating xyz from pcd: {pcd_path}")
    filename = pathlib.Path(pcd_path).stem
    xyz_path = os.path.join(output_path, filename + ".xyz")

    # cmd = f"{CC_BIN} -o {pcd_path} -C_EXPORT_FMT ASC -DENSITY 0.1 -TYPE SURFACE -ROUGHNESS 0.1"
    cmd = f"{PCD_TO_XYZ_BIN}  {pcd_path} {xyz_path} 0"
    print(f"running command: {cmd}")
    os.system(cmd)
    return xyz_path


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])

    if not os.path.exists(args.i):
        raise Exception(f"Invalid input path: {args.i}")

    extension = pathlib.Path(args.i).suffix
    if extension == ".pcd":
        xyz_path = pcd_to_xyz(args.i, args.o)
    elif extension == ".xyz":
        xyz_path = args.i
    else:
        raise Exception(f"Invalid filetype: {args.i}")

    plot_statistics(xyz_path, args.o)
