clc, clear

% Evan McLaughlin
% 2018-03-19
% Generate the compliment of a set of images
% Ensure the directory is correct and then run the code to generate the 
% compliment of the images
images = imageDatastore(fullfile(pwd,'rotated/flir_histeq'));
imageFileNames = images.Files;

for i = 1:length(imageFileNames)
    imageA = imread(imageFileNames{i});
    imageB = imcomplement(imageA(:,:,1));
    if i < 10
        imageName = strcat('image0',num2str(i),'.jpg');
    else
        imageName = strcat('image',num2str(i),'.jpg');
    end
    %imshowpair(imageA,imageB,'montage')
    imwrite(imageB, fullfile(pwd,'rotated/flir_complement',imageName));
end
