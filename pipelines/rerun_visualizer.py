
import argparse
import os
import json
from pypcd4 import PointCloud
import numpy as np
import rerun as rr
from pathlib import Path
from scipy.spatial.transform import Rotation as R
from typing import Dict
import cv2

JPEG_QUALITY = 0.75

'''
Installing requirements for this viewer:

pip3 install rerun-sdk pypcd4 opencv-python
(for some reason, this wasn't working so I did a venv and installed projectaria_tools)

Example command:

python3 pipelines/rerun_visualizer.py \
--maps ~/d2/results/map_builder_refined/map_lidar_v_link.pcd \
~/d2/results/map_builder_unrefined/map_lidar_v_link.pcd \
--map_names refined_map unrefined_map \
--trajectories ~/d2/results/map_builder_refined/final_poses.json \
~/d2/results/map_builder_unrefined/final_poses.json \
--trajectory_names refined_trajectory unrefined_trajectory \
--extrinsics pipelines/calibrations/extrinsics_reorg.json \
--intrinsics ~/beam_robotics/calibration/results/inspector_gadget2/current/intrinsics \
--cameras_list_path ~/d2/results/image_extractor/CameraListNew.json
'''


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--maps",
        type=str,
        nargs="+",
        help="path(s) to map files (pcd)",
    )
    parser.add_argument(
        "--map_names",
        type=str,
        nargs="+",
        help="names associate with each map file",
    )
    parser.add_argument(
        "--trajectories",
        nargs="+",
        type=str,
        help="path(s) to trajectories. Trajectories should be in the 'baselink' frame",
    )
    parser.add_argument(
        "--trajectory_names",
        nargs="+",
        type=str,
        help="names associated with each trajectory file",
    )
    parser.add_argument(
        "--extrinsics",
        type=str,
        help="path to the extrinsics.json file",
    )
    parser.add_argument(
        "--intrinsics",
        type=str,
        help="path to folder that has the intrinsics json files",
    )
    parser.add_argument(
        "--cameras_list_path",
        type=str,
        help="path to cameras list json, see ImageExtractor output. "
        "Image poses are taken from first trajectory file",
    )

    return parser.parse_args()


def log_map(map_path: str, map_name: str, world_frame: str):
    print(f"reading map: {map_path}")
    pc: PointCloud = PointCloud.from_path(map_path)
    pc_numpy = pc.numpy()
    print(f"loaded map {map_name} with {pc_numpy.shape[0]} points")
    entity_path = (
        f"{world_frame}/{map_name}"
    )
    rr.log(
        entity_path,
        rr.Points3D(pc_numpy, radii=0.005),
        static=True,
    )


def load_trajectory(trajectory_path: str) -> Dict[int, np.array]:
    print(
        f"loading trajectory {trajectory_path} to get image poses")
    with open(trajectory_path, 'r') as f:
        trajectory = json.load(f)
    poses = {}
    for pose in trajectory["poses"]:
        transform = pose["transform"]
        T = np.eye(4)
        for i in range(4):
            for j in range(4):
                T[i][j] = transform[i*4 + j]
        s = pose["time_stamp_sec"]
        nsec = int(pose["time_stamp_nsec"])
        timestamp_ns = nsec + int(s * 1e9)
        poses[timestamp_ns] = T
    return poses


def log_trajectory(trajectory_path: str, trajectory_name: str):
    print(f"loading trajectory {trajectory_name} from {trajectory_path}")
    with open(trajectory_path, 'r') as f:
        trajectory = json.load(f)
    world_frame = trajectory["fixed_frame"]
    poses = []
    for pose in trajectory["poses"]:
        T = pose["transform"]
        poses.append([T[3], T[7], T[11]])
    print(f"loaded {len(poses)} poses")
    rr.log(
        f"{world_frame}/{trajectory_name}",
        rr.LineStrips3D(poses, radii=0.008),
        static=True,
    )


def get_frames_from_trajectory(trajectory_path: str) -> str:
    with open(trajectory_path, 'r') as f:
        trajectory = json.load(f)
    world_frame = trajectory["fixed_frame"]
    baselink_frame = trajectory["moving_frame"]
    return world_frame, baselink_frame


def log_extrinsics(extrinsics_path: str, world_frame: str, baselink_frame: str):
    print(f"logging extrinsics from {extrinsics_path}")
    with open(extrinsics_path, 'r') as f:
        extrinsics = json.load(f)

    frames_to_ignore = ["thermal_link", "vicon/SDICIG2/SDICIG2"]
    for calibration in extrinsics["calibrations"]:
        from_frame = calibration["from_frame"]
        if from_frame in frames_to_ignore:
            continue
        entity_path = f"{world_frame}/{baselink_frame}/{from_frame}"
        T = calibration["transform"]
        T_Baselink_Sensor = np.eye(4)
        for i in range(4):
            for j in range(4):
                T_Baselink_Sensor[i][j] = T[i*4 + j]
        r = R.from_matrix(T_Baselink_Sensor[:3, :3])
        t = T_Baselink_Sensor[:3, 3]
        T_rr = rr.Transform3D(
            translation=t,
            rotation=rr.Quaternion(xyzw=r.as_quat()),
            from_parent=False,
        )
        rr.log(entity_path, T_rr, static=True)


def log_image(image_container_path: str, world_frame: str, baselink_frame: str, trajectory: Dict[int, np.array]):
    # get image path
    viz_image_path = os.path.join(image_container_path, "BGRMaskOverlay.jpg")
    if os.path.exists(viz_image_path):
        image_path = viz_image_path
    else:
        image_path = os.path.join(image_container_path, "BGRImage.jpg")

    if not os.path.exists(image_path):
        print(f"WARNING: image not found at {image_path}")
        return

    # get timestamp
    image_info_path = os.path.join(image_container_path, "ImageInfo.json")
    assert os.path.exists(
        image_info_path), f"image info json not available at: {image_info_path}"
    with open(image_info_path, 'r') as f:
        image_info = json.load(f)
    camera_frame_id = image_info["bgr_frame_id"]
    timestamp_ns = image_info["time_stamp"]

    rr.set_time_nanos("device_time", timestamp_ns)
    rr.set_time_sequence("timestamp", timestamp_ns)

    T_World_Baselink = find_closest_pose(trajectory, timestamp_ns)
    r = R.from_matrix(T_World_Baselink[:3, :3])
    t = T_World_Baselink[:3, 3]
    T_rr = rr.Transform3D(
        translation=t,
        rotation=rr.Quaternion(xyzw=r.as_quat()),
    )
    rr.log(f"{world_frame}/{baselink_frame}", T_rr)

    img = cv2.imread(image_path)
    img_np = np.array(img)
    rr.log(
        f"{world_frame}/{baselink_frame}/{camera_frame_id}",
        rr.Image(img_np),
    )


def find_closest_pose(trajectory: Dict[int, np.array], timestamp_ns: int) -> np.array:
    dt_smallest_ns = 1e10
    closest_timestamp = -1
    for t in trajectory.keys():
        dt = abs(t - timestamp_ns)
        if dt < dt_smallest_ns:
            dt_smallest_ns = dt
            closest_timestamp = t
    return trajectory[closest_timestamp]


def log_images(cameras_list_path: str, intrinsics_directory: str, world_frame: str, baselink_frame: str, trajectory: Dict[int, np.array]):
    # get cameras and iterate through them
    with open(cameras_list_path) as f:
        camera_list = json.load(f)

    # log camera models
    cameras = camera_list["Cameras"]
    camera_to_frame_id = {}
    for camera in cameras:
        intrinsics_filename = camera[:-5]
        intrinsics_path = os.path.join(
            intrinsics_directory, f"{intrinsics_filename}.json")
        assert os.path.exists(
            intrinsics_path), f"could not find extrinsics for camera {camera} at {intrinsics_path}"
        with open(intrinsics_path, 'r') as f:
            intrinsics = json.load(f)
        camera_frame = intrinsics["frame_id"]
        camera_to_frame_id[camera] = camera_frame
        w = intrinsics["image_width"]
        h = intrinsics["image_height"]
        f = intrinsics["intrinsics"][0]
        rr.log(
            f"{world_frame}/{baselink_frame}/{camera_frame}",
            rr.Pinhole(
                resolution=[w, h],
                focal_length=float(f),
            ),
            static=True,
        )

    images_filename = camera_list["ImagesFilename"]
    cameras_path = Path(cameras_list_path).parent

    # log images
    for camera in cameras:
        images_path = os.path.join(cameras_path, camera)
        image_list_path = os.path.join(images_path, f"{images_filename}.json")
        with open(image_list_path, 'r') as f:
            images = json.load(f)["Items"]
            for img_name in images:
                image_path = os.path.join(images_path, img_name)
                log_image(image_path, world_frame, baselink_frame, trajectory)


def main():
    args = parse_args()

    assert len(args.maps) == len(
        args.map_names), "number of maps much match number of map names"
    assert len(args.trajectories) == len(
        args.trajectory_names), "number of trajectories much match number of trajectory names"

    for filepath in args.maps:
        assert os.path.exists(filepath), f"file does not exist: {filepath}"
    for filepath in args.trajectories:
        assert os.path.exists(filepath), f"file does not exist: {filepath}"
    assert os.path.exists(
        args.extrinsics), f"file does not exist: {args.extrinsics}"

    world_frame, baselink_frame = get_frames_from_trajectory(
        args.trajectories[0])

    rr.init("Beam Inspection Viewer", spawn=True)
    rr.log(world_frame, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    trajectory_first = load_trajectory(args.trajectories[0])

    log_extrinsics(args.extrinsics, world_frame, baselink_frame)

    for trajectory, trajectory_name in zip(args.trajectories, args.trajectory_names):
        log_trajectory(trajectory, trajectory_name)

    for map, map_name in zip(args.maps, args.map_names):
        log_map(map, map_name, world_frame)

    log_images(args.cameras_list_path, args.intrinsics,
               world_frame, baselink_frame, trajectory_first)


if __name__ == "__main__":
    main()
