#!/usr/bin/env python
import argparse
import rosbag
import rospy
import sys

from rosbag import Bag

def main(args):

    parser = argparse.ArgumentParser(
        description='This script can be used to add or remove a constant time offset from a specific topic. It will then resave the bag to a new name')
    parser.add_argument('--bagfile', nargs=1, help='input bag file')
    parser.add_argument('--output', nargs=1,  help='output bag file')
    parser.add_argument('--topic', nargs=1,  help='topic to offset timestamps')
    parser.add_argument('--offset', nargs=1,  type=float, help='time offset in seconds')
    args = parser.parse_args()

    bagfile = args.bagfile[0]
    output = args.output[0]
    topic = args.topic[0]
    offset = args.offset[0]

    rostime_offset = rospy.rostime.Duration.from_sec(offset)

    with rosbag.Bag(output, 'w') as outbag:
        for msg_topic, msg, t in rosbag.Bag(bagfile).read_messages():
            if msg_topic == topic:
                new_msg = msg
                new_msg.header.stamp = msg.header.stamp + rostime_offset
                outbag.write(msg_topic, new_msg, new_msg.header.stamp)
            elif msg_topic != '/diagnostics':
                outbag.write(msg_topic, msg, t)

if __name__ == "__main__":
    main(sys.argv[1:])
