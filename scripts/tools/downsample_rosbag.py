#!/usr/bin/env python
import argparse
import rosbag
import rospy
import sys

from rosbag import Bag


def search(list, item):
    for i in range(len(list)):
        if list[i] == item:
            return True, i
    return False, -1


def downsample_rosbag(bag_in_name, bag_out_name, topics, skips, start_offset, end_offset):
    bag_in = rosbag.Bag(bag_in_name)
    bag_out = rosbag.Bag(bag_out_name, 'w')

    new_bag_start_time = rospy.Time.from_sec(
        bag_in.get_start_time()) + rospy.Duration.from_sec(start_offset)
    new_bag_end_time = rospy.Time.from_sec(
        bag_in.get_end_time()) - rospy.Duration.from_sec(end_offset)

    counters = [0] * len(topics)

    for topic, msg, t in bag_in.read_messages():

        # only filter messages that have a header, but filter based on index time
        if msg._has_header:
            if t < new_bag_start_time or t > new_bag_end_time:
                continue
        else:
            if t < new_bag_start_time:
                bag_out.write(topic, msg, new_bag_start_time)
            elif t > new_bag_end_time:
                bag_out.write(topic, msg, new_bag_end_time)
            else:
                bag_out.write(topic, msg, t)    
            continue

        topic_found, index = search(topics, topic)
        if topic_found:
            if (int(counters[index]) >= int(skips[index])):
                counters[index] = 0
                bag_out.write(topics[index], msg, t)
            else:
                counters[index] = counters[index] + 1

        else:
            bag_out.write(topic, msg, t)

    bag_out.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Downsample a ros bag by editing start and end times of all topics (using indexed '
                    'time), and skipping every N messages for specific topics. Example usage: '
                    'python downsample_rosbag.py --bag_in bag_in_path.bag --bag_out '
                    'bag_out_path.bag --downsample_topics \'/topic1 /topic1\' --downsample_counts '
                    '\'1 1\' --start_offset 5 --end_offset 10')
    parser.add_argument('--bag_in', required=True,
                        nargs=1, help='input bag file')
    parser.add_argument('--bag_out', required=True,
                        nargs=1, help='output bag file')
    parser.add_argument('--downsample_topics', required=True, default="*",
                        help='topics to downsample, string interpreted as a list of topics')
    parser.add_argument('--downsample_counts', required=True, default="*",
                        help='number of messages to skip in output bag for above topics. '
                        'This needs to be the same size as downsample_topics')
    parser.add_argument('--start_offset', required=False, nargs=1, type=float,
                        default=[0], help='start time offset in seconds for all messages')
    parser.add_argument('--end_offset', required=False, nargs=1, type=float,
                        default=[0], help='end time offset in seconds for all messages')
    args = parser.parse_args()

    topics = args.downsample_topics.split(' ')
    skips = args.downsample_counts.split(' ')
    if(len(topics) != len(skips)):
        print('ERROR: Size of downsample_topics and downsample_counts must be equal!')
    else:
        try:
            downsample_rosbag(args.bag_in[0], args.bag_out[0], topics,
                              skips, args.start_offset[0], args.end_offset[0])
        except Exception, e:
            import traceback
            traceback.print_exc()
