import torch
from PIL import Image
import os
import json
import numpy as np
import argparse

from torchvision.transforms import transforms as T
from ModelUtils.crack_segmentation_model import pretrained_mask_rcnn

from keras.models import load_model
from ModelUtils.delam_segmentation_model import relu6, BilinearUpsampling
from ModelUtils.deep_defect_functions import *


# load models
def main(data_path, config_path, crack, delam, corrosion, spall, visualize, verbose):
    print("\n")
    if not config_path.endswith(".json"):
        print("ERROR: Config file needs to be .json")
        print("Config File:", config_path)
        return

    if not os.path.isfile(config_path):
        print("ERROR: Config file does not exist")
        print("Config File Path Given:", config_path)
        return

    # load config file
    with open(config_path) as config_json:
        config_dict = json.load(config_json)
        if crack:
            crack_model_path = config_dict["crack_model_path"]

            if not os.path.isabs(crack_model_path):
                crack_model_path = os.path.abspath(crack_model_path)

            if not os.path.isfile(crack_model_path):
                print("ERROR: Crack model file does not exist. Crack segmentation will not be completed")
                crack = False
            num_classes = config_dict["crack_num_classes"]  # default = 2, crack and background classes 
            crack_confidence_threshold = config_dict["crack_confidence_threshold"]  # default = 0.7
            crack_seg_threshold = config_dict["crack_seg_threshold"]  # default = 0.3

        if delam:
            delam_model_path = config_dict["delam_model_path"]
            
            if not os.path.isabs(delam_model_path):
                delam_model_path = os.path.abspath(delam_model_path)

            if not os.path.isfile(delam_model_path):
                print("ERROR: Delam model file does not exist. Delam segmentation will not be completed")
                delam = False

        if corrosion:
            corrosion_model_path = config_dict["corrosion_model_path"]

            if not os.path.isabs(corrosion_model_path):
                corrosion_model_path = os.path.abspath(corrosion_model_path)

            if not os.path.isfile(corrosion_model_path):
                print("ERROR: Corrosion model file does not exist. Corrosion segmentation will not be completed")
                corrosion = False

        if spall:
            spall_model_path = config_dict["spall_model_path"]

            if not os.path.isabs(spall_model_path):
                spall_model_path = os.path.abspath(spall_model_path)

            if not os.path.isfile(spall_model_path):
                print("ERROR: Spall model file does not exist. Spall segmentation will not be completed")
                spall = False

    # load models
    if crack:
        crack_model = pretrained_mask_rcnn(num_classes)

        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        if torch.cuda.is_available():
            crack_model.cuda()
        crack_checkpoint = torch.load(crack_model_path, map_location=device)
        crack_model.load_state_dict(crack_checkpoint["model"])
        crack_model.eval()
        crack = False
        try:
            crack_checkpoint = torch.load(crack_model_path, map_location=device)
            crack_model.load_state_dict(crack_checkpoint["model"])
            crack_model.eval()
            crack = False
        except:
            print("ERROR: Cannot load crack model file. Crack segmentation will not be completed")       

    if delam:
        print("load delam model")
        try:
            delam_model = load_model(delam_model_path,
                                     custom_objects={'relu6':relu6,
                                     'BilinearUpsampling':BilinearUpsampling,
                                     'perDelam':perDelam,
                                     'predDelam':predDelam,
                                     'realDelam':realDelam,
                                     'iou_loss':iou_loss})
        except:
            print("ERROR: Cannot load delam model file. Delam segmentation will not be completed")
            delam = False

    if corrosion:
        print("load corrosion model")

    if spall:
        print("load spall model")


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
                if verbose: 
                    print(data_path + camera + "/" + img_name + "/BGRImage.jpg")
                img = Image.open(data_path + camera + "/" + img_name + "/BGRImage.jpg").convert("RGB")
                width, height = img.size
                if width > 3000 or height > 2000:
                    new_size = (int(width/2), int(height/2))
                    img = img.resize(new_size)

                if crack:
                    crack_masks = []
                    img_tensor = [T.ToTensor()(img)]

                    with torch.no_grad():
                        output = crack_model(img_tensor)[0]
                        for i in range(len(output["scores"])):
                            if output["scores"][i] > crack_confidence_threshold:
                                mask = output["masks"][i]
                                mask = np.where(mask > crack_seg_threshold, 1, 0).astype(np.uint8)
                                crack_masks.append(mask)

                                if "mask_rcnn_crack" not in mask_methods:
                                    mask_methods.append("mask_rcnn_crack")

                    crack_mask = (np.logical_or.reduce(crack_masks)).astype(np.uint8)
                    crack_mask = np.transpose(crack_mask[0], (1, 0))
                # spalling et al detection here

                #combining mask layers
                bgr_base = np.zeros((img.size[1], img.size[0]))
                
                if corrosion:
                    idx = np.where(corrosion_mask != 0)
                    bgr_base[idx[1], idx[0]] = 3
                if crack:
                    idx = np.where(crack_mask != 0)
                    bgr_base[idx[1], idx[0]] = 1
                if spall:
                    idx = np.where(spall_mask != 0)
                    bgr_base[idx[1], idx[0]] = 4

                mask_img = Image.fromarray(np.transpose(np.stack([bgr_base, bgr_base, bgr_base], axis=1), [0, 2, 1]).astype(np.uint8))
                mask_img = mask_img.resize((width, height))
                mask_img.save(data_path + camera + "/" + img_name + "/BGRMask.jpg")

                if visualize:
                    bgr_base *= 50  # largest label is 4 will correspond to 200
                    mask_img_visualize = Image.fromarray(np.transpose(np.stack([bgr_base, bgr_base, bgr_base], axis=1), [0, 2, 1]).astype(np.uint8))
                    mask_img_visualize = mask_img_visualize.resize((width, height))
                    mask_img_visualize.save(data_path + camera + "/" + img_name + "/BGRMask_visualize.jpg")

                img_info["bgr_mask_method"] = ", ".join(mask_methods)
                img_info["is_bgr_mask_set"] = True

            if img_info["is_ir_image_set"]:
                if verbose: 
                    print(data_path + camera + "/" + img_name + "/IRImage.jpg")
                
                if delam:
                    img = np.asarray(Image.open(data_path + camera + "/" + img_name + "/IRImage.jpg"))
                    
                    img_scale = customRescale(img)

                    res = delam_model.predict(np.expand_dims(img_scale,0), batch_size=1)
                    labels = np.argmax(res.squeeze(),-1)
                    ir_mask = Image.fromarray((labels).astype(np.uint8))
                    ir_mask.save(data_path + camera + "/" + img_name + "/IRMask.jpg")

                    if visualize:
                        mask_visualize = Image.fromarray((labels*255).astype(np.uint8))
                        mask_visualize.save(data_path + camera + "/" + img_name + "/IRMask_visualize.jpg")

                    img_info["ir_mask_method"] = "Deeplabv3_delam"
                    img_info["is_ir_mask_set"] = True

            with open(data_path + camera + "/" + img_name + "/ImageBridgeInfo.json", "w") as img_json:
                json.dump(img_info, img_json)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('data_path')
    parser.add_argument('config', type=str)
    parser.add_argument('--crack', action='store_false')
    parser.add_argument('--delam', action='store_false')
    parser.add_argument('--corrosion', action='store_false')
    parser.add_argument('--spall', action='store_false')
    parser.add_argument('--vis', action='store_true')
    parser.add_argument('--verbose', action='store_true')

    args = parser.parse_args()
    main(args.data_path, args.config, args.crack, args.delam, args.corrosion, args.spall, args.vis, args.verbose)
