import rosbag
import sys
import argparse
import os

from restamp import *


def main(args):
    parser = argparse.ArgumentParser(
        description='Plot TimeReference msgs to validate synchronization.')
    parser.add_argument('-b', '--bag', nargs=1, help='input bag file')
    parser.add_argument(
        '-d', '--data_topics', nargs='+', help='whitespace separated list of topics')
    parser.add_argument(
        '-t', '--time_topics', nargs='+', help='whitespace separated list of topics')

    args = parser.parse_args()

    bag = rosbag.Bag(args.bag[0])
    folder = os.path.dirname(args.bag[0])
    outfile = os.path.join(folder, "output.bag")
    outbag = rosbag.Bag(outfile, "w")

    time_topics = args.time_topics
    data_topics = args.data_topics
    
    # pair and restamp data/time message couples
    bag = restamp(bag, data_topics, time_topics)
    
    # print(bag.read_messages())
    # for topic, msg, t in bag.read_messages(data_topics):
    #     if not topic in msgs:
    #         msgs[topic] = []
    #         stamps[topic] = []
    #     msgs[topic].append(msg)
    #     stamps[topic].append(
    #         rospy.Time(msg.time_ref.secs, msg.time_ref.nsecs).to_sec())

    # bag.close()
    # print(range(len(stamps[topics[0]])))
    # print(stamps[topics[0]])
    # plt.scatter(range(len(stamps[topics[0]])), stamps[topics[0]])
    # plt.show()


if __name__ == "__main__":
    main(sys.argv[1:])
