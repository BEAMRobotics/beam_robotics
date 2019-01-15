% Auto-generated by cameraCalibrator app on 19-Mar-2018
%-------------------------------------------------------
clc, clear, close all
% Generated by Evan McLaughlin, edited by Nick Charron
% To reuse the code:
% Update the input information
% Run the file to calibrate and generate an output text file
% Output includes the intrinsic matrix and radial distortion coefficients
% plus other useful information

% Input Information:
    captureDate = '2018_04_05';
    imDir = '/home/nick/data/ximea_calib/2018_04_05/';
    imType = '*.jpg';
    userName = 'Nick Charron';
    squareSize = 7.620000e+01;  % in units of 'millimeters'

% Read images and store directories as a structure
    imageFileNames = {};
    imLookup = fullfile(imDir, imType);
    imFiles = dir(imLookup);
    for i = 1:length(imFiles)
        imageFileNames{i} = fullfile(imFiles(i).folder,imFiles(i).name);
    end

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates of the corners of the squares
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', true, ...
    'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams);

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% Define intrinsic matrix and radial distortion matrix
int_mat = cameraParams.IntrinsicMatrix;
rad_dist = cameraParams.RadialDistortion;
tan_dist = cameraParams.TangentialDistortion;

% Print to text file
fid = fopen(fullfile('..', 'results' ,[captureDate, '_ximea_calibration','.txt']), 'wt');
fprintf(fid, 'Ximea Calibration Results:\n');
fprintf(fid, '--------------------------\n');
fprintf(fid, ['\nImage Capture Date: ', captureDate]);
fprintf(fid, ['\nDone by: ', userName]); 
fprintf(fid, '\n\nIntrinsic Matrix:\n');
for i = 1:3
    for j = 1:3
        if j == 3
            fprintf(fid, '%12.6f\n', int_mat(i,j));
        else
            fprintf(fid, '%12.6f', int_mat(i,j));
        end
    end
end
fprintf(fid, '\nRadial Distortion\n');
fprintf(fid, '%12.7f', rad_dist);
fprintf(fid, '\n\nTangential Distortion\n');
fprintf(fid, '%12.7f', tan_dist);
fprintf(fid, '\n\nMean Reprojection Error: ');
fprintf(fid, '%.4f', cameraParams.MeanReprojectionError);
fprintf(fid, '\n\nImages Used:\n');
for i = 1:length(imFiles)
    fprintf(fid, [imFiles(i).name,'\n']); 
end
fclose(fid);
