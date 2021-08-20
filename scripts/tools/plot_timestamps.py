import rosbag
import rospy
import sys
import argparse
import matplotlib.pyplot as plt


def main(args):
    parser = argparse.ArgumentParser(
        description='Plot header timestamps to validate synchronization.')
    parser.add_argument('-b', '--bag', nargs=1, help='input bag file')
    parser.add_argument(
        '-t', '--topics', nargs='+', help='whitespace separated list of topics')

    args = parser.parse_args()

    bag = rosbag.Bag(args.bag[0])
    topics = args.topics

    msgs = {}
    stamps = {}

    for topic, msg, t in bag.read_messages(topics):
        if not topic in msgs:
            msgs[topic] = []
            stamps[topic] = []
        msgs[topic].append(msg)
        stamps[topic].append(
            rospy.Time(msg.stamp.secs, msg.stamp.nsecs).to_sec())

    bag.close()
    print(range(len(stamps[topics[0]])))
    print(stamps[topics[0]])
    plt.scatter(range(len(stamps[topics[0]])), stamps[topics[0]])
    plt.show()


if __name__ == "__main__":
    main(sys.argv[1:])
