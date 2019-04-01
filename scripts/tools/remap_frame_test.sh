#!/bin/bash

set -e # exit on first error

# Test cases for path coverage of 'remap_frames.py'
# The following shell script iteratively calls 'remap_frame_v2.py' with different raw input arguments to check for bugs on all paths

# Case 1 - Single valid topic/rename frame with valid input

python ./remap_frames_test.py <<EOF
/home/inspector-gadget/bag_files/ig_scans/ig_scan_2019-02-14-12-39-13_short.bag
/home/inspector-gadget/bag_files/ig_scans/ig_scan_2019-02-14-12-39-13_short_A.bag
/hvlp/velodyne_points
h_points1+?
EOF
echo "Case 1 complete: Single valid topic with valid rename"

# Case 2 - Single invalid topic

python ./remap_frames_test.py <<EOF
/home/inspector-gadget/bag_files/ig_scans/ig_scan_2019-02-14-12-39-13_short.bag
/home/inspector-gadget/bag_files/ig_scans/ig_scan_2019-02-14-12-39-13_short_A.bag
/hvlp/invalid_topic1+?
h_points
EOF
echo "Case 2 complete: Single invalid topic"

# Case 3 - Multiple (2) valid topics/rename frames with valid inputs

python ./remap_frames_test.py <<EOF
/home/inspector-gadget/bag_files/ig_scans/ig_scan_2019-02-14-12-39-13_short.bag
/home/inspector-gadget/bag_files/ig_scans/ig_scan_2019-02-14-12-39-13_short_A.bag
/hvlp/velodyne_points,/vvlp/velodyne_points
h_v_points
v_v_points
EOF
echo "Case 3 complete: Multiple (2) valid topics with valid renames"

# Case 4 - Multiple topics (2) (1) valid (1) invalid

python ./remap_frames_test.py <<EOF
/home/inspector-gadget/bag_files/ig_scans/ig_scan_2019-02-14-12-39-13_short.bag
/home/inspector-gadget/bag_files/ig_scans/ig_scan_2019-02-14-12-39-13_short_A.bag
/hvlp/velodyne_points,/vvlp/invalid_topic1+?
h_v_points
v_v_points
EOF
echo "Case 4 complete: Multiple (2) topics (1) valid and (1) invalid with valid renames"
