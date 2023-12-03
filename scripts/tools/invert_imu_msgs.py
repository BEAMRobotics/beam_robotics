#!/usr/bin/env python

import sys
import rosbag
import argparse

from rosbag import Bag


def main(args):

    parser = argparse.ArgumentParser(
        description='multiplies imu data by -1 and saves as /imu/data_inverted')
    parser.add_argument('--bagfile', nargs=1, help='input bag file')
    parser.add_argument('--output', nargs=1,
                        help='output bag file', default=['output.bag'])
    args = parser.parse_args()

    bagfile = args.bagfile[0]

    with rosbag.Bag(args.output[0], 'w') as outbag:
        for topic, msg, t in rosbag.Bag(bagfile).read_messages():
            # copy all original messages
            outbag.write(topic, msg, t)

            # invert imu data and add to new topic
            if topic == "/imu/data":
                msg_inv = msg
                msg_inv.linear_acceleration.x = - 1 * msg.linear_acceleration.x
                msg_inv.linear_acceleration.y = - 1 * msg.linear_acceleration.y
                msg_inv.linear_acceleration.z = - 1 * msg.linear_acceleration.z
                outbag.write("/imu/data_inverted", msg_inv, t)


if __name__ == "__main__":
    main(sys.argv[1:])
