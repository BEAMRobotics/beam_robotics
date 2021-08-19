import torch
import glob
from PIL import Image
import os
import json
import numpy as np

from torchvision.transforms import transforms as T
from crack_segmentation_model import pretrained_mask_rcnn
import sys


# load models

def main(data_path, config_path):
    print(config_path)
    if not config_path.endswith(".json"):
        print("ERROR: Config file needs to be .json")
        print("Config File:", config_path)
        return

    if not os.path.isfile(config_path):
        print("ERROR: Config file does not exist")
        print("Config File Path Given:", config_path)
        return

    with open(config_path) as config_json:
        config_dict = json.load(config_json)
        crack_model_path = config_dict["crack_model_path"]
        num_classes = config_dict["num_classes"]  # crack and background classes
        crack_confidence_threshold = config_dict["crack_confidence_threshold"]
        crack_seg_threshold = config_dict["crack_seg_threshold"]
    crack_model = pretrained_mask_rcnn(num_classes)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    if torch.cuda.is_available():
        crack_model.cuda()

    crack_checkpoint = torch.load(crack_model_path, map_location=device)
    crack_model.load_state_dict(crack_checkpoint["model"])
    crack_model.eval()

    # spall model

    # get cameras and iterate through them
    with open(data_path + "CamerasList.json") as camera_json:
        camera_list = json.load(camera_json)["Items"]

    for camera in camera_list:
        with open(data_path + camera + "/ImagesList.json") as imgs_json:
            imgs_list = json.load(imgs_json)["Items"]

        for img_name in imgs_list:
            with open(data_path + camera + "/" + img_name + "/ImageBridgeInfo.json") as img_json:
                img_info = json.load(img_json)

            if img_info["is_bgr_image_set"]:
                mask_methods = []
                print(data_path + camera + "/" + img_name + "/BGRImage.jpg")
                img = Image.open(data_path + camera + "/" + img_name + "/BGRImage.jpg").convert("RGB")
                width, height = img.size
                if width > 3000 or height > 2000:
                    new_size = (int(width/2), int(height/2))
                    img = img.resize(new_size)
                img_tensor = [T.ToTensor()(img)]

                masks = []
                with torch.no_grad():
                    output = crack_model(img_tensor)[0]
                    for i in range(len(output["scores"])):
                        if output["scores"][i] > crack_confidence_threshold:
                            mask = output["masks"][i]
                            mask = np.where(mask > crack_seg_threshold, 1, 0).astype(np.uint8)
                            masks.append(mask)
                            mask *= 255

                            if "mask_rcnn" not in mask_methods:
                                mask_methods.append("mask_rcnn")

                # spalling et al detection here

                if len(masks) > 0:
                    mask = (np.logical_or.reduce(masks) * 255).astype(np.uint8)
                    mask_img = Image.fromarray(np.transpose(np.stack([mask, mask, mask], axis=1)[0], [1, 2, 0]))
                    mask_img = mask_img.resize((width, height))
                    mask_img.save(data_path + camera + "/" + img_name + "/BGRMask.jpg")
                else:
                    mask_img = Image.new('RGB', (width, height))
                    mask_img.save(data_path + camera + "/" + img_name + "/BGRMask.jpg")

                img_info["bgr_mask_method"] = ", ".join(mask_methods)
                img_info["is_bgr_mask_set"] = True

            with open(data_path + camera + "/" + img_name + "/ImageBridgeInfo.json", "w") as img_json:
                json.dump(img_info, img_json)


if __name__ == "__main__":
    data_path = sys.argv[1]
    config_file_path = sys.argv[2]
    main(data_path, config_file_path)
