#!/usr/bin/env python
import argparse
import rosbag
import rospy
import sys

from rosbag import Bag


def offset_message_timestamps(bag_in, bag_out, topics, offset):

    bag_out = rosbag.Bag(bag_out, 'w')
    time_offset = rospy.Duration.from_sec(offset)

    for topic, msg, t in rosbag.Bag(bag_in).read_messages():
        # for now, skip messages without a header, we will copy these later
        if not msg._has_header:
            continue
    
        if topics[0] == "":
            msg.header.stamp = msg.header.stamp + time_offset
            bag_out.write(topic, msg, t + time_offset)
            continue

        topic_found, index = search(topics, topic)
        if topic_found:
            msg.header.stamp = msg.header.stamp + time_offset
            bag_out.write(topics[index], msg, t + time_offset)
        else:
            bag_out.write(topic, msg, t)

    # iterate back thgit srough the bag now that we know the start and end times and update 
    bag_start_time = rospy.Time.from_sec(bag_out.get_start_time()) 
    bag_end_time = rospy.Time.from_sec(bag_out.get_end_time()) 
    for topic, msg, t in rosbag.Bag(bag_in).read_messages():
        if msg._has_header:
            if t < bag_start_time:
                bag_out.write(topic, msg, bag_start_time)
            elif t > bag_end_time:
                bag_out.write(topic, msg, bag_end_time)
            else:
                bag_out.write(topic, msg, t)
    bag_out.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Add or remove a constant time offset to specified topics within a bagfile. A'
                    'new bag file is written containing all of the original topics in the input '
                    'bag, but specified topics will have all message timestamps and index times offset.')
    parser.add_argument('--bag_in', required=True,
                        nargs=1, help='input bag file')
    parser.add_argument('--bag_out', required=True,
                        nargs=1, help='output bag file')
    parser.add_argument('--topics', required=False, default="",
                        help='topics to offset message timestamps, string interpreted as a list '
                             'of topics. If not specified, it will offset all msgs with timstamps')
    parser.add_argument('--offset', required=True, nargs=1,
                        type=float, help='time offset in seconds')

    args = parser.parse_args()

    topics = args.topics.split(' ')

    try:
        offset_message_timestamps(
            args.bag_in[0], args.bag_out[0], topics, args.offset[0])
    except Exception, e:
        import traceback
        traceback.print_exc()
