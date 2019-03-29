#!/usr/bin/env python
import sys
import os
from os.path import join

import cv2
import rosbag
from cv_bridge import CvBridge


def print_usage():
    print("Usage: bag2imgs.py <ros bag> <ros topic> <output path>")
    print("Example: bag2imgs.py record.bag /robot/camera robot_images/")


if __name__ == "__main__":
    # Check CLI args
    if len(sys.argv) != 4:
        print_usage()
        exit(-1)

    # Parse CLI args
    bag = rosbag.Bag(sys.argv[1], 'r')
    topic = sys.argv[2]
    output_path = sys.argv[3]

    # Create output directory
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    # Check if topic is in bag
    info = bag.get_type_and_topic_info()
    if topic not in info.topics:
        raise RuntimeError("Opps! topic not in bag!")

    # Check image message type
    msg_type = info.topics[topic].msg_type
    supported_msgs = ["sensor_msgs/CompressedImage", "sensor_msgs/Image"]
    if msg_type not in supported_msgs:
        err_msg = "bag2vid only supports %s!" % " or ".join(supported_msgs)
        raise RuntimeError(err_msg)

    # Convert bag to images
    br = CvBridge()
    index = 0
    for topic, msg, t in bag.read_messages(topics=[topic]):
	if index%1 == 0: # change setting to only take every x images         
		# Convert image message to np.array
        	if msg_type == "sensor_msgs/CompressedImage":
            		image = br.compressed_imgmsg_to_cv2(msg)
        	else:
            		image = br.imgmsg_to_cv2(msg)

        	# Write image to file
        	image_fname = join(output_path, "image_%d.jpg" % index)
        	cv2.imwrite(image_fname, image)
        index += 1
