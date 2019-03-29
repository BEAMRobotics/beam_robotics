#!/usr/bin/env python
import rosbag
from copy import deepcopy
import tf

bagInName = '/home/nick/bag_files/ig_scans/2018_12_ATS/ig_scan_2018-12-12-17-15-11.bag'
print("reading bag in...")
bagIn = rosbag.Bag(bagInName)
bagOutName = '/home/nick/bag_files/ig_scans/2018_12_ATS/ig_scan_2018-12-12-17-15-11_A.bag'
print("opening bag out...")
bagOut = rosbag.Bag(bagOutName,'w')
with bagOut as outbag:
    print("iterating through bag ...")
    for topic, msg, t in bagIn.read_messages():
        if topic == '/hvlp/velodyne_points' or topic == '/vvlp/velodyne_points':
            new_msg = deepcopy(msg)
            if msg.header.frame_id == "velodyne_vert":
                new_msg.header.frame_id = "vvlp_link"
            if msg.header.frame_id == "velodyne_horz":
                new_msg.header.frame_id = "hvlp_link"
            outbag.write(topic, new_msg, t)
        else:
            outbag.write(topic, msg, t)
print("Closing bags...")
bagIn.close()
bagOut.close()
print("Done.")
