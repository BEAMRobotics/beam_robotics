Instructions for ximea intrinsic calibration:
By: Evan McLaughlin
Date: 2018-03-19


* For all codes ensure that the directory stated to be used to search for the
photos is correct for where you have saved the photos to be used for
calibration. See individual MATLAB codes for more information.

1.  Run the camera calibration on the ximea photos. If the right directory for
    the photos has been specified, the program will output a .txt file
    containing the intrinsic and radial distortion matrices for the ximea.
2. Output file Format:
	- The camera intrinsic matrix has the following format:
		
		| fx  0  0 |
		| s   fy 0 |
		| cx  cy 1 |

	- the radial distortion params are

		[ k1  k2  k3]

	  Using the following model:

		x_dist = x(1 + k1 r^2 + k2 r^4 + k3 r^6)
		y_dist = y(1 + k1 r^2 + k2 r^4 + k3 r^6)

	- the radial distortion params are

		[ p1 p2]

	  Using the following model:

		x_dist = x + [2 p1 y + p2 (r^2 + 2 x^2)]
		y_dist = y + [p1 (r^2 + 2 y^2) + 2 p2 x]

This is based on matlab calibration docs: https://www.mathworks.com/help/vision/ref/cameraparameters.html
