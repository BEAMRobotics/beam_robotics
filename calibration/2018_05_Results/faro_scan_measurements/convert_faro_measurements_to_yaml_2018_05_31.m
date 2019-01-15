%% ------------------------------------------------------------------------
% Description:
%   The Yaml files required by the ig_calibration_publisher need specific
%   transforms to complete the tf tree. Since the faro scan measurements
%   were done by defining different transforms, we need to convert that.
%   Also, the faro scans were in mm which needs to be converted to m
%
% Yaml File Transforms:
%   T_BASELINK_HVLP
%   T_HVLP_GPS
%   T_VVLP_HVLP
%   T_XIMEA_VVLP
%   T_FLIR_XIMEA
%   T_IMU_XIMEA
%
% Faro Scan Measurement Results:
%   T_BASELINK_HVLP 
%   T_HVLP_GPS
%   T_HVLP_VVLP
%   T_HVLP_XIMEA
%   T_HVLP_IMU
%   T_FLIR_XIMEA: Not measured - Use stereo calib. results
%
%-------------------------------------------------------------------------
clc; clear;
%% Inputting measured transforms:
f = 1000;
T_BASELINK_HVLP = [ 1   0   0	200.1/f
                    0   1   0	 32.5/f
                    0   0   1	324.8/f
                    0   0   0	   1]
T_HVLP_GPS = [  1	0	0	-164.5/f
                0	1	0	 161.0/f
                0	0	1	 228.6/f
                0	0	0	   1]
% t_HVLP_IMU = [  -16/f
%                  22.4/f
%                 -165.5/f
%                  1];
t_HVLP_IMU = [ -165.5/f
                 -16/f
                 22.4/f
                 1];             
R_roll_minus90 = eul2rotm([0 0 pi/2]);
R_yaw_minus90 = eul2rotm([pi/2 0 0]);
R_HVLP_IMU = R_yaw_minus90 * R_roll_minus90;
T_HVLP_IMU = [[R_HVLP_IMU; [0 0 0]] , t_HVLP_IMU]

t_HVLP_VVLP = [ -245.6/f 
                 0/f 
                 124.4/f
                 1];
R_HVLP_VVLP = eul2rotm([0 -pi/2 0]);             
T_HVLP_VVLP = [ [R_HVLP_VVLP; [0 0 0] ], t_HVLP_VVLP ]


t_HVLP_XIMEA = [ -112.6/f
                 -49.2/f
                 177.0/f
                 1];
R_HVLP_XIMEA = eul2rotm([-pi/2 0 0]);             
T_HVLP_XIMEA = [[R_HVLP_XIMEA; [0 0 0]],t_HVLP_XIMEA]

T_XIMEA_FLIR = [    0.99997   0.00522   0.00549  -0.07353
                   -0.00540   0.99948   0.03177   0.00919
                   -0.00532  -0.03180   0.99948   0.00570
                    0.00000   0.00000   0.00000   1.00000];

%% Calculating required transforms

T_VVLP_HVLP = inv(T_HVLP_VVLP)

T_XIMEA_VVLP = inv(T_HVLP_XIMEA) * T_HVLP_VVLP

T_IMU_XIMEA = inv(T_HVLP_IMU) * T_HVLP_XIMEA

T_FLIR_XIMEA = inv(T_XIMEA_FLIR) 

%% Write Results to Yaml File

fid = fopen('./results/2018_05_31.yaml', 'wt');
transforms = {T_BASELINK_HVLP, T_HVLP_GPS, T_VVLP_HVLP, T_XIMEA_VVLP, T_FLIR_XIMEA, T_IMU_XIMEA};
transform_names = {'T_BASELINK_HVLP', 'T_HVLP_GPS', 'T_VVLP_HVLP', 'T_XIMEA_VVLP', 'T_FLIR_XIMEA', 'T_IMU_XIMEA'};

for k = 1:size(transforms,2)
    fprintf(fid, '%s: ', transform_names{k});
    for i = 1:4
        for j = 1:4
            fprintf(fid, '%10.5f', transforms{k}(i,j));
        end
    end
    fprintf(fid, '\n');
end
fclose(fid);
