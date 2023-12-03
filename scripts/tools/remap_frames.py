#!/usr/bin/env python
import rosbag
from copy import deepcopy
import tf

bagInName = raw_input(
    "Enter the path to the bag you want to edit the headers for: ")
print("reading bag in..")
bagIn = rosbag.Bag(bagInName)
bagOutName = raw_input("Enter the path and new name for the edited bag: ")
print("opening bag out...")
bagOut = rosbag.Bag(bagOutName, 'w')
target_topics = raw_input(
    "Enter the topics you wish to rename a frame for (separated by a comma): ").split(',')
target_frame_out = []
for position in range(len(target_topics)):
    target_frame_out.append(raw_input(
        "What would you like to rename the frame of " + target_topics[position] + " to? "))
with bagOut as outbag:
    print("iterating through bag ...")
    for topic, msg, t in bagIn.read_messages():
        if topic in target_topics:
            new_msg = deepcopy(msg)
            ind = target_topics.index(topic)
            new_msg.header.frame_id = target_frame_out[ind]
            print("Successfully renamed " + msg.header.frame_id +
                  " to " + new_msg.header.frame_id)
            outbag.write(topic, new_msg, t)
        else:
            outbag.write(topic, msg, t)
print("Closing bags...")
bagIn.close()
bagOut.close()
print("Done.")
