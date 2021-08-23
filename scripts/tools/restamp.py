import rosbag
import sys
import argparse
import os

def restamp(bag, outbag, data_topics, time_topics):
    if (len(data_topics) != len(time_topics)):
        raise Exception("Length of time and data topic arguments is not the same.")
    

def main(args):
    parser = argparse.ArgumentParser(
        description='Plot TimeReference msgs to validate synchronization.')
    parser.add_argument('-b', '--bag', nargs=1, help='input bag file')
    parser.add_argument(
        '-d', '--data_topics', nargs='+', help='whitespace separated list of topics')
    parser.add_argument(
        '-t', '--time_topics', nargs='+', help='whitespace separated list of topics')

    args = parser.parse_args()

    data_topics = args.data_topics
    time_topics = args.time_topics
  
    bag = rosbag.Bag(args.bag[0])
    folder = os.path.dirname(args.bag[0])
    outfile = os.path.join(folder, "output.bag")
    outbag = rosbag.Bag(outfile, "w")

    stamped = restamp(bag, outbag, data_topics, time_topics)

if __name__ == "__main__":
    main(sys.argv[1:])