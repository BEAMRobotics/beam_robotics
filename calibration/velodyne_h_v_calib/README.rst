Overview
========

This package can be used to tweak the calibration between two lidars.
Simply launch the launch file, run a bag with the "--clock" param and vary
the sliders until your scans line up.

Detailed Instructions:

- if you haven't already done this, you may need to install rqt ez publisher:

  sudo apt-get install ros-kinetic-rqt-ez-publisher

  - make sure your launch file has the correct yaml file for initial calibration

  - after launching the launch file and the rosbag, change the topic on the top
  dropdown menu to: /velodyne_calib_tf

  - you can set the minimum and maximum values for each translation and rotation
  so that the sliders become more or less sensitive.

  - to save the final calibration, check the checkbox at the bottom left. It
  will automatically save to a yaml file in the calibration publisher package
  which will automatically be used by the calibration publisher.

  - You can also save the current configuration for the sliders. To do this: hit
  the gear icon and select "save to file".

  - IMPORTANT NOTE: Since we are not using a proper YAML emitter here, you will
  need to go open your final yaml calibration file in any text editor and resave
  it. No need to edit anything or rename it, simply open, press save then close.
