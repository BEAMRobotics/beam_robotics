import rosbag
import rospy
import sys
import subprocess
import yaml
import argparse
import matplotlib.pyplot as plt


def main(args):
    parser = argparse.ArgumentParser(
        description='Plot all TimeReference topics in a bag. This is helpful to validate synchronization in a dataset (e.g., ig2 data).')
    parser.add_argument('-b', '--bag', nargs=1, help='input bag file', required=True)
    parser.add_argument(
        '-t', '--topics', nargs='+', help='whitespace separated list of topics to include, leave empty for all TimeReference topics')

    args = parser.parse_args()

    bag = rosbag.Bag(args.bag[0])

    topics = list()
    if args.topics is not None:
        for topic in args.topics:
            if topic == "sensor_msgs/TimeReference":
                topics.append(topic)
    else:
        info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', args.bag[0]], stdout=subprocess.PIPE).communicate()[0])
        for topic in info_dict["topics"]:
            if topic["type"] == "sensor_msgs/TimeReference":
                topics.append(topic["topic"])


    stamps = {}

    for topic, msg, t in bag.read_messages(topics):
        if not topic in stamps:
            stamps[topic] = []
        stamps[topic].append(msg.time_ref.to_sec())

    bag.close()

    fig = plt.figure()
    ax = fig.add_subplot(111)
    for topic in stamps:
        ax.scatter(x=range(len(stamps[topic])), y=stamps[topic], label=topic)
    plt.legend(loc="lower right")
    plt.show()


if __name__ == "__main__":
    main(sys.argv[1:])
