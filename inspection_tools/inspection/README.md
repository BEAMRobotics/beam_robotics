# Inspection

This module contains the main coded needed to perform an inspection given a ROS bag and SLAM resutls (i.e. final pcd map and a poses file).

## Architecture and Design

This software has been designed with a main program and sub-routine architecture (or call and return type architecture). Therefore, in src you will find main files with sub-routines defined in include/inspection and implementations in src/lib.

Main files:

 * AutoInspection.cpp: This will implement fully automated inspection, including automated image labeling.
 * ManualInspectionA.cpp: This is the first of two parts needed to run to perform semi-automated inspection. This will require manual image labeling, however the rest of the process should be fully automated.
 * ManualInspectionB.cpp: This is the second part to the above.

## Running Inspections

Each main file is an executable. Once the code is built, you can run the executables directly from the terminal. First, make sure you set your configuration files (.json files) in config.

The automated inspection exectuable can be run on its own. If performing the semi-automated inspection, you will need to run these in sequence:

  1. run ManualInspectionA
  2. label images exported during 1
  3. run ManualInspectionB

## Image Extractor Instructions  

Set all the parameters in ImageExtractorConfig.json located in the config directory.
For each image topic, you have to specify the same number of variables for the
following parameters:

 * distance_between_images : minimum change in distance between extracted images
 * rotation_between_images : minimum change in rotation between extracted images
 * are_images_distorted : bool of wether or not the image have been undistorted
 * image_enhancing : some image processing method have been implemented (see below)

### Image Enhancing:

There are currently 3 options for enhancing the images at extraction time.

 * none : do not edit images (it will still debayer and save as jpeg)
 * linear: linear enhancer g(i,j)=α⋅f(i,j)+β, where α = gain, and β = bias
 * histogram: histogram equalization according to:
