Instructions for stereo calibration:
By: Evan McLaughlin
Date: 2018-03-19


* For all codes ensure that the directory stated to be used to search for the
photos is correct for where you have saved the photos to be used for
calibration. See individual MATLAB codes for more information.

1.  Resize ximea photos to match FLIR photos using ximeaResize.m
      - The photos must be the same pixel size to do the calibration
2.  Histogram equalize FLIR photos using flirHisteq.m
      - This enhances contrast to better detect IR checkerboard
3.  Generate compliment of FLIR photos using flirCompliment.m
      - This inverts the colours on the checkerboard so that the flir images
        are consistend with the ximea images
4.  Run the stereo calibration on the resized ximea photos and histogram equalized
    and compliment FLIR photos. Ensure the ximea camera is camera 1. If the
    code is run properly the program will output a .txt file containing the
    transformation matrix between the cameras.
