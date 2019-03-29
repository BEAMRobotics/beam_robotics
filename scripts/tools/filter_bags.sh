#!/bin/bash

set -e # exit on first error

echo "Enter the (path to)/(name of) your bag file: "
read bagfile

echo "Enter the (path to)/(name of) of the new bag file: "
read newbagfile

echo "Filtering bag: $bagfile and saving to $newbagfile ..."

rosbag filter  $bagfile $newbagfile \
"topic != '/F2/image_color/compressed' and \
topic != '/F1/image_color/compressed'"
