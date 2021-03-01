#!/usr/bin/env python
import argparse
import rosbag
import rospy
import sys

from rosbag import Bag

def offset_message_timestamps(bag_in, topics, offset, postfix):

    print('Offsetting...'),
    bag_out = rosbag.Bag(bag_in[:-4] + '_' + postfix + '.bag', 'w')
    time_offset = rospy.Duration.from_sec(offset)

    for topic, msg, t in rosbag.Bag(bag_in).read_messages():
        bag_out.write(topic, msg, t)
        if topic in topics:
            if topic == "/tf":
                for transform in msg.transforms:
                    transform.header.stamp += time_offset
            elif msg._has_header:
                msg.header.stamp += time_offset

            bag_out.write(topic + '_' + postfix, msg, t + time_offset)

    bag_out.close()
    print('complete')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Add or remove a constant time offset to specified topics within a bagfile. A'
                    'new bag file is written containing all of the original topics in the input '
                    'bag, with the addition of offset topics demarcated with a postfix.')
    parser.add_argument('--bag_in', required=True, nargs=1, help='input bag file')
    parser.add_argument('--topics', required=True, nargs='+', help='topics to offset message timestamps')
    parser.add_argument('--offset', required=True, nargs=1, type=float, help='time offset in seconds')
    parser.add_argument('--postfix', required=False, nargs=1, default='offset', 
                        help='postfix to demarcate output bag file and time-offset topics')
    args = parser.parse_args()
    try:
        offset_message_timestamps(args.bag_in[0], args.topics, args.offset[0], args.postfix)
    except Exception, e:
        import traceback
        traceback.print_exc()

