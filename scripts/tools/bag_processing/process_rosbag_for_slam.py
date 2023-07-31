import os
import sys
import argparse
import rosbag

import restamp_raw_bag

def restamp_bag(input, output):
    time_topics = ["/imu/imu_time", "/F1/cam_time"]
    data_topics = ["/imu/data", "/F1/image_raw"]
    print("restamping bag using time_topics = ", str(time_topics), " data_topics = ", str(data_topics))
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
    os.system('rm -rf ' + bag_backup)

def preprocess_bag(input, output, calibration):
    bin_dir=os.path.join(os.path.expanduser("~"), 'catkin_ws/build/rosbag_tools/process_bag')
    cmd = bin_dir + ' --input ' + input + ' --output ' + output 
    cmd += ' --unpack_scans=1 --debayer_images=1 --rectify_images=0 --lidar_model VLP16 '
    cmd += '--camera_model_path ' + calibration + ' --resize_multiplier 0.25'
    print('running command: ', cmd)
    os.system(cmd)

def cleanup_bag(input, output):
    print('cleaning up bag and saving as: ', output)
    cmd = 'rosbag filter ' + input + ' ' + output + ' '
    cmd += 'topic == '
    bagin = rosbag.Bag(input, "r")
    bagout = rosbag.Bag(output, "w")

    for topic, msg, t in bagin.read_messages():
        if topic == '/F1/image_raw_resized':
            bagout.write('/F1/image', msg, t)
        elif topic == '/imu/data':
            bagout.write(topic, msg, t)    
        elif topic == '/lidar_h/velodyne_packets_unpacked':
            bagout.write('/lidar_h/velodyne_points', msg, t)        

    bagin.close()
    bagout.close()


def main(args):
    parser = argparse.ArgumentParser(description='Tool for preprocessing an IG2 bag to run through Beam SLAM. \n'
                                     'Steps are: \n'
                                     '  (1) restamp raw bag to assign hardware sync timestamps [input.bag -> tmp_bag_restamped.bag]\n'
                                     '  (2) run rosbag_tools/ProcessBag to unpack scans, debayer & resize images [tmp_bag_restamped.bag -> tmp_bag_preprocessed.bag]\n'
                                     '  (3) remove unused topics and rename remaining topics\n'
                                     'Note: this script expects your rosbag_tools to be in ~/catkin_ws/build/')
    parser.add_argument('-b', '--raw_bag', help='Input raw bag file. ')
    parser.add_argument('-o', '--output_bag', help='Output bag file')
    parser.add_argument('-c', '--calibration', help='Calibration json file for F1')
    parser.add_argument('--keep_intermediate_files', action='store_true')
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
    preprocess_bag(output_restamped, output_preprocessed, args.calibration)
    cleanup_bag(output_preprocessed, args.output_bag)

    if not args.keep_intermediate_files:
        os.system('rm -rf ' + output_restamped)
        os.system('rm -rf ' + output_preprocessed)


if __name__ == "__main__":
    main(sys.argv[1:])
