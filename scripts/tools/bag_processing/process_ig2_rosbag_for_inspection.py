import os
import sys
import argparse
import rosbag

import restamp_raw_bag


def restamp_bag(input, output):
    time_topics = ["/imu/imu_time", "/F1/cam_time",
                   "/F2/cam_time", "/F3/cam_time", "/F4/cam_time"]
    data_topics = ["/imu/data", "/F1/image_raw",
                   "/F2/image_raw", "/F3/image_raw", "/F4/image_raw",]
    print("restamping bag using time_topics = ", str(
        time_topics), " data_topics = ", str(data_topics))
    print("saving to bag: ", output)
    inbag = rosbag.Bag(input, "r")
    outbag = rosbag.Bag(output, "w")
    restamp_raw_bag.restamp(inbag, outbag, data_topics, time_topics)
    outbag.close()
    inbag.close()

    print("re-indexing bag")
    os.system('rosbag reindex ' + output)
    bag_backup = output[:-4] + '.orig.bag'
    print('removing: ', bag_backup)
    os.remove(bag_backup)


def preprocess_bag(input, output):
    bin_dir = os.path.join(os.path.expanduser(
        "~"), 'catkin_ws/build/rosbag_tools/process_bag')
    cmd = bin_dir + ' --input ' + input + ' --output ' + output
    cmd += ' --unpack_scans=1 --debayer_images=1 --rectify_images=0 --lidar_model VLP16 '
    cmd += '--compress_images=1'
    print('running command: ', cmd)
    os.system(cmd)


def cleanup_bag(input, output):
    print('cleaning up bag and saving as: ', output)
    cmd = 'rosbag filter ' + input + ' ' + output + ' '
    cmd += 'topic == '
    bagin = rosbag.Bag(input, "r")
    bagout = rosbag.Bag(output, "w")

    for topic, msg, t in bagin.read_messages():
        if topic.find("_raw_resized") != -1:
            new_topic = topic[:-len("_raw_resized")]
            bagout.write(new_topic, msg, t)
        elif topic.find("_raw_debayered") != -1:
            new_topic = topic[:-len("_raw_debayered")]
            bagout.write(new_topic, msg, t)
        elif topic.find("_packets_unpacked") != -1:
            new_topic = topic[:-len("packets_unpacked")] + "points"
            bagout.write(new_topic, msg, t)
        elif topic == '/imu/data':
            bagout.write(topic, msg, t)

    bagin.close()
    bagout.close()


def main(args):
    parser = argparse.ArgumentParser(description='Tool for preprocessing an IG2 bag to run through Beam Robotics Inspection pipeline. \n'
                                     'Steps are: \n'
                                     '  (1) restamp raw bag to assign hardware sync timestamps [input.bag -> tmp_bag_restamped.bag]\n'
                                     '  (2) run rosbag_tools/ProcessBag to unpack scans, debayer & resize images [tmp_bag_restamped.bag -> tmp_bag_preprocessed.bag]\n'
                                     '  (3) remove unused topics and rename remaining topics\n'
                                     'Note: this script expects your rosbag_tools to be in ~/catkin_ws/build/')
    parser.add_argument('-b', '--raw_bag', help='Input raw bag file. ')
    parser.add_argument('-o', '--output_bag', help='Output bag file')
    parser.add_argument('--keep_intermediate_files', action='store_false')
    args = parser.parse_args()

    if not os.path.exists(args.raw_bag[0]):
        print("ERROR: input bag path invalid: ", args.raw_bag)
        exit()

    output_dir = os.path.dirname(args.output_bag)

    if not os.path.exists(output_dir):
        print("ERROR: output bag path invalid: ", args.output_bag)
        exit()

    output_restamped = os.path.join(output_dir, 'tmp_bag_restamped.bag')
    output_preprocessed = os.path.join(output_dir, 'tmp_bag_preprocessed.bag')

    restamp_bag(args.raw_bag, output_restamped)
    preprocess_bag(output_restamped, output_preprocessed)
    cleanup_bag(output_preprocessed, args.output_bag)

    if not args.keep_intermediate_files:
        os.remove(output_restamped)
        os.remove(output_preprocessed)


if __name__ == "__main__":
    main(sys.argv[1:])
