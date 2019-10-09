import sys
import rosbag
import time
import subprocess
import yaml
import rospy
import os
import argparse
import math
import warnings
from shutil import move

def status(length, percent):
  sys.stdout.write('\x1B[2K') # Erase entire current line
  sys.stdout.write('\x1B[0E') # Move to the beginning of the current line
  progress = "Reordering Bag: ["
  for i in range(0, length):
    if i < length * percent:
      progress += '='
    else:
      progress += ' '
  progress += "] " + str(round(percent * 100.0, 2)) + "% complete"
  sys.stdout.write(progress)
  sys.stdout.flush()

def main(args):

  parser = argparse.ArgumentParser(description='Reorder a bagfile based on header timestamps.')
  parser.add_argument('bagfile', nargs=1, help='input bag file')
  parser.add_argument('--output', nargs=1, help='output bag file', default=['output.bag'])
  parser.add_argument('--max-offset', nargs=1, help='max time offset (sec) to correct.', default='5', type=float)
  args = parser.parse_args()

  bagfile = args.bagfile[0]

  info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bagfile], stdout=subprocess.PIPE).communicate()[0])
  duration = info_dict['duration']
  start_time = info_dict['start']

  with rosbag.Bag(args.output[0], 'w') as outbag:

    last_time = time.clock()
    topic_list = []

    for topic, msg, t in rosbag.Bag(bagfile).read_messages():

      # set time bar
      if time.clock() - last_time > .1:
          percent = (t.to_sec() - start_time) / duration
          status(40, percent)
          last_time = time.clock()

      # This also replaces tf timestamps under the assumption
      # that all transforms in the message share the same timestamp
      if topic == "/tf" and msg.transforms:
        # Writing transforms to bag file 1 second ahead of time to ensure availability
        diff = math.fabs(msg.transforms[0].header.stamp.to_sec() - t.to_sec())
        outbag.write(topic, msg, msg.transforms[0].header.stamp - rospy.Duration(1) if diff < args.max_offset else t)
      elif msg._has_header:
        diff = math.fabs(msg.header.stamp.to_sec() - t.to_sec())
        outbag.write(topic, msg, msg.header.stamp if diff < args.max_offset else t)
      else:
        diff = None
        outbag.write(topic, msg, t)

      if diff > args.max_offset and topic not in topic_list:
        topic_list.append(topic)

  status(40, 1)

  print "\nThe following topics fell outside of the max time offset:"
  for i in range(len(topic_list)):
    print "  " + topic_list[i]

if __name__ == "__main__":
  main(sys.argv[1:])
