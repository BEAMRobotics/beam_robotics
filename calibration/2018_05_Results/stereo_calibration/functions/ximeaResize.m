clc, clear
% Evan McLaughlin
% 2018-03-19
% Resize Ximea photos to be the same size as the FLIR photos
% Ensure the directory is correct and run the code to generate resized photos

% Create a set of calibration images.
images = imageDatastore(fullfile(pwd,'original/ximea'));
imageFileNames = images.Files;

for i = 1:length(imageFileNames)
    imageA = imread(imageFileNames{i});
    imageB = imresize(imageA,0.5);
    if i < 10
        imageName = strcat('image0',num2str(i),'.jpg');
    else
        imageName = strcat('image',num2str(i),'.jpg');
    end
    %imshowpair(imageA,imageB,'montage')
    imwrite(imageB, fullfile(pwd,'original/ximeaResize',imageName));
end
