# Inspection

This module contains the main code needed to perform an inspections given a ROS bag and SLAM resutls. There are different modules that need to be called in sequence with the correct input data. Below you will find a description of the executables as well as some important instructions.

## TODO:

 * Create pipelines (in python?) that call all the executables to run the whole inspection pipeline from start to finish.
 * Automate defect labeling in images. Currently we have some CNN models trained for spall detection and delamination detection (in IR images) in beam_robotics/inspection_tools/cnn_defect_segmentation. But these models cannot yet be called from within our inspection pipeline. We need to create an executable that applies these models to the images from inspection_extract_images. The models were created using TensorFlow so it might be easier to make this a python executable, but it also could be convenient to be able to call Pytorch models which can easily be done in C++. We also have ImageLabeler class files stored here as a place holder but they are empty.
 * Create GenerateReport class. Currently the files are in place but empty. The goal of this class would be to get use the defect quantities to generate a report of this inspection.

## Executables:

 * inspection_extract_images: This takes a bag file and a set of poses and extracts images at certain intervals based on motion. Images are extracted into image containers with the option of extracting them all to the same root directory (not using image containers). 
 * inspection_label_map: This takes the output from SLAM (map + poses) and a set of images from inspection_extract_images which also have defect masks, and returns a labeled map. The map has the same set of points from the SLAM map but each point has a label for each of the defect classes.
 * inspection_quantify_defects: This takes the results from the label_map and quantifies all the defects.
 * odom_topic_to_poses_file: This is a tool for getting a poses file from an odometry topic in a ROS bag. 

 For more information on running any of these executables, run the executable with the argument --help

## Running Inspections

The inspection codebase has a set of executables that need to be run in sequence. You can run the executables in order directly from the terminal to go through the whole inspection pipeline. First, make sure you set your configuration files (.json files) in config.

The order at which to run the executables to go through the whole inspection pipeline are as follows:

  1. Run SLAM. SLAM can be run using a variety of SLAM tools, but none of the SLAM code lives in inspection. 
  2. Export SLAM map to a PCD file
  3. Generate a json poses file (you can use odom_topic_to_poses_file if the SLAM method chosen does not output the poses file)
  4. Run inspection_extract_images given the original ROS bag and the poses file
  5.  Label the images from inspection_extract_images by generating defect masks. This step has not been fully automated.

## Image Extractor Instructions  

Set all the parameters in ImageExtractorConfig.json located in the config directory. For each image topic, you have to specify the same number of variables for the following parameters:

 * distance_between_images : minimum change in distance between extracted images
 * rotation_between_images : minimum change in rotation between extracted images
 * are_images_distorted : bool of wether or not the image have been undistorted
 * image_enhancing : some image processing method have been implemented (see below)

### Image Enhancing:

There are currently 3 options for enhancing the images at extraction time.

 * none : do not edit images (it will still debayer and save as jpeg)
 * linear: linear enhancer g(i,j)=α⋅f(i,j)+β, where α = gain, and β = bias
 * histogram: histogram equalization
