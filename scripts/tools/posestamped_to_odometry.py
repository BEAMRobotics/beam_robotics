#!/usr/bin/env python
import argparse
import rosbag
import rospy
import sys

from rosbag import Bag
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def convert_single_message(pose_stamped_msg, child_frame_id):
    odometry_msg = Odometry()
    odometry_msg.header = pose_stamped_msg.header
    odometry_msg.child_frame_id = child_frame_id
    odometry_msg.pose.pose = pose_stamped_msg.pose
    return odometry_msg

def convert_messages(bag_in, bag_out, topic_in, topic_out, child_frame_id):
    print('Converting topic: ' + topic_in)
    print('in bag: ' + bag_in)
    print('writing to bag: ' + bag_out)
    print('on topic: ' + topic_out)
    print('with child frame id: ' + child_frame_id)

    bag_out = rosbag.Bag(bag_out, 'w')
    counter = 0
    for topic, msg, t in rosbag.Bag(bag_in).read_messages():
        # convert 
        if topic_in == topic:
            new_message = convert_single_message(msg, child_frame_id)
            bag_out.write(topic_out, new_message, t)
            counter = counter + 1
        
        # copy all messages
        bag_out.write(topic, msg, t)

    bag_out.close()

    if counter == 0:
        print('ERROR: no messages found with topic: ', topic_in)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='This script takes a bag and converts some topic from geometry_msgs/PoseStamped to '
                    'nav_msgs/Odometry and saves all original bag messages along with new messages.')
    parser.add_argument('--bag_in', required=True,
                        nargs=1, help='input bag file')
    parser.add_argument('--bag_out', required=True,
                        nargs=1, help='output bag file')
    parser.add_argument('--topic_in', required=True,
                        help='topic of geometry_msgs/PoseStamped to be converted.')
    parser.add_argument('--topic_out', required=True,
                        help='output topic for new nav_msgs/Odometry message.')
    parser.add_argument('--child_frame_id', required=True,
                        help='child frame id for odometry message. PoseStamped only has frame_id which '
                        'is the baselink frame. The child frame id should be the fixed frame (map or '
                        'world or odom).')                        

    args = parser.parse_args()

    try:
        convert_messages(
            args.bag_in[0], args.bag_out[0], args.topic_in, args.topic_out, args.child_frame_id)
    except Exception, e:
        import traceback
        traceback.print_exc()
