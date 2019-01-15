clc, clear

% Evan McLaughlin
% 2018-03-19
% Histogram equalize a set of images
% Ensure the directory is correct and run the code to histogram equalize
% the images
images = imageDatastore(fullfile(pwd,'original/flir'));
imageFileNames = images.Files;

for i = 1:length(imageFileNames)
    imageA = imread(imageFileNames{i});
    imageB = histeq(imageA(:,:,1));
    if i < 10
        imageName = strcat('image0',num2str(i),'.jpg');
    else
        imageName = strcat('image',num2str(i),'.jpg');
    end
    %imshowpair(imageA,imageB,'montage')
    imwrite(imageB, fullfile(pwd,'original/flirHisteq',imageName));
end
