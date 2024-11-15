import os

import tensorflow as tf
import numpy as np
import os
import cv2
import time
import pickle as cPickle
import skimage.draw
from PIL import Image

import yolo.config as cfg
from yolo.yolo3_net_pos_tfv2 import YOLONet
from utils.voc_eval_mask import voc_eval

# tf.compat.v1.disable_eager_execution()
yolo_net = YOLONet(False)

MODEL_WEIGHTS_DIR = '/home/nick/data/dis_yolo/pretrained_weights'

CAMERA_LIST = "/home/nick/data/2021_10_07_11_01_30_KitchenerParkingGarage/results/image_extractor/CameraListNew.json"


def main():
    print("Running Dis-Yolo Inference")
    weights_file_data = os.path.join(
        CAMERA_LIST, "yolov3_3class_coco.ckpt.data")
    weights_file_meta = os.path.join(
        CAMERA_LIST, "yolov3_3class_coco.ckpt.meta")

    print("Dis-Yolo inference completed successfully")


if __name__ == "__main__":
    main()
