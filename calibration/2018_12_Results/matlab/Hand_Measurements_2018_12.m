%% Converting Hand Measurements Taken during Dec 2018 Calibration
clc; clear; format short;

% T_BASELINK_HVLP:

t_BASELINK_HVLP = [21; 0; 35.2]*10^(-2); % m
R_BASELINK_HVLP = eye(3);
T_BASELINK_HVLP = buildTransform(R_BASELINK_HVLP,t_BASELINK_HVLP)

% T_X1_HVLP

t_X1_HVLP =  [-8 ; 0 ; -4]*10^(-2); % m
R_X1_HVLP = eul2rotm([0 -pi/2 0]); % DEFAULT IS ZYX
T_X1_HVLP = buildTransform(R_X1_HVLP, t_X1_HVLP)

% T_X1_IMU1

t_X1_IMU1 = [0; 0; -9]*10^(-2); % m
R_X1_IMU1 = eul2rotm([0 0 pi/2])*eul2rotm([0 -pi/2 0]);
T_X1_IMU1 = buildTransform(R_X1_IMU1, t_X1_IMU1)

% T_HVLP_VVLP

t_HVLP_VVLP = [-13.5; 0; 12]*10^(-2); % m
R_HVLP_VVLP = eul2rotm([0 0 pi]) * eul2rotm([0 pi/2 0]);
T_HVLP_VVLP = buildTransform(R_HVLP_VVLP, t_HVLP_VVLP)

% T_VVLP_F2

t_VVLP_F2 = [5; 10; -6]*10^(-2); % m
R_VVLP_F2 = eul2rotm([pi -pi/2 20*pi/180]);
T_VVLP_F2 = buildTransform(R_VVLP_F2, t_VVLP_F2)

% T_F2_F3

t_F2_F3 = [-1; 10.6; -3.4]*10^(-2); % m
R_F2_F3 = eul2rotm([0 0 -20*pi/180]);
T_F2_F3 = buildTransform(R_F2_F3, t_F2_F3)

% T_F2_F1

t_F2_F1 = [0; 21.2; -6.8]*10^(-2); % m
R_F2_F1 = eul2rotm([0 0 -40*pi/180]);
T_F2_F1 = buildTransform(R_F2_F1, t_F2_F1)

% T_HVLP_GPS

t_HVLP_GPS = [-57; -20; 23]*10^(-2); % m
R_HVLP_GPS = eye(3);
T_HVLP_GPS = buildTransform(R_HVLP_GPS, t_HVLP_GPS)

% T_IMU2_HVLP

t_IMU2_HVLP = [-11; -20; 6.5]*10^(-2); % m
R_IMU2_HVLP = eul2rotm([pi/2 -pi/2 0]);
T_IMU2_HVLP = buildTransform(R_IMU2_HVLP, t_IMU2_HVLP)

% T_F3_IMU2

t_F3_IMU2 = [46; -11; 32.5]*10^(-2); % m
R_F3_IMU2 = eul2rotm([-pi/2 0 -pi/2]);
T_F3_IMU2 = buildTransform(R_F3_IMU2, t_F3_IMU2)

% T_F1_IMU2

% These measurements were wrong : too hard to measure directly by hand
    % t_F1_IMU2 = [13.5; -0.5; -41.3]*10^(-2); % m
    % R_F1_IMU2 = eul2rotm([0 0 20*pi/180]) * eul2rotm([0 pi/2 0]) * eul2rotm([-pi/2 0 0]);
    % T_F1_IMU2 = buildTransform(R_F1_IMU2, t_F1_IMU2)
% Alernate method to calculate:
    T_F1_IMU2 = inv(T_F2_F1) * inv(T_VVLP_F2) *  inv(T_HVLP_VVLP) * inv(T_IMU2_HVLP)
    %T_F1_IMU2_B = inv(T_F2_F1) * T_F2_F3 * T_F3_IMU2 % <-- wrong

% T_F2_IMU1

t_F2_IMU1 = [14; 1; -23]*10^(-2); % m
R_F2_IMU1 = eul2rotm([0 0 -20*pi/180]) * eul2rotm([-90*pi/180 0 0]);
T_F2_IMU1 = buildTransform(R_F2_IMU1, t_F2_IMU1)

% T_F1_IMU1

T_F1_IMU1 = inv(T_F2_F1) * T_F2_IMU1

%% For LOAM

quat_BASELINK_HVLP = rotm2quat(R_BASELINK_HVLP);
quat_HVLP_BASELINK = rotm2quat(R_BASELINK_HVLP');
T_HVLP_BASELINK = inv(T_BASELINK_HVLP);
T_BASELINK_HVLP;

R_BASELINK_BASELINKROT = eul2rotm([-pi/2 -pi/2 0]); % DEFAULT IS ZYX
quat_BASELINK_BASELINKROT = rotm2quat(R_BASELINK_BASELINKROT);

%% function for combining t and R into T 

function T = buildTransform(R,t) 
    T = [ [R,t] ; [0 0 0 1]];
    I_test = round(R*R',3);
    if (I_test == eye(3))
        % good
    else
        disp('ERROR: NOT A VALID ROTATION MATRIX')
    end
end