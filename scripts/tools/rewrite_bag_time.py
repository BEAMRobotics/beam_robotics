import sys
import rosbag
import argparse

def main(args):

    parser = argparse.ArgumentParser(description='Update the bag time based on tf stamp. '
    'This is useful if the ROS messages have different timestamps as the ROS time (clock)'
    ' if for example you have rerecorded the bag.')
    parser.add_argument('--bagfile', nargs=1, help='input bag file')
    parser.add_argument('--output', nargs=1,  help='output bag file', default=['output.bag'])
    args = parser.parse_args()

    bagfile = args.bagfile[0]

    with rosbag.Bag(args.output[0], 'w') as outbag:
        for topic, msg, t in rosbag.Bag(bagfile).read_messages():
            # This also replaces tf timestamps under the assumption
            # that all transforms in the message share the same timestamp
            if topic == "/tf" and msg.transforms:
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                outbag.write(
                    topic, msg, msg.header.stamp if msg._has_header else t)

if __name__ == "__main__":
    main(sys.argv[1:])
