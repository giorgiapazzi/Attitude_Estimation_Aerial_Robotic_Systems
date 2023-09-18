close all
% clear all
clc

% Load dataset
load("all_data_in_ws.mat")


% Extract real data IMU sensor
acc_IMU = ts_imu_acc.Data; % accelerometer measurements
gyro_IMU = ts_imu_gyro.Data;   % gyroscope measurements
mag_IMU = ts_imu_mag.Data; % magnetometer measurements
numsamples = size(acc_IMU,1);   % total number of samples
fs = 100;

% Convert data from imu frame to wand frame
% Compute Cimu^wand matrix:
% define IMU mounting:
%  - the IMU X axis is aligned with -Y wand (vicon) axis
%  - the IMU Y axis is aligned with -X wand (vicon) axis 
% thus the rotation from IMU axis to wand_axis is : 
% first Rz(90), then Ry(180) 

acc_IMU_wand = zeros(numsamples,3);
gyro_IMU_wand = zeros(numsamples,3);
mag_IMU_wand = zeros(numsamples,3);

for i = 1 : size(acc_IMU,1)
    acc_IMU_wand(i,:) = (r_imu_mount * (acc_IMU(i,:)'))';
    gyro_IMU_wand(i,:) = (r_imu_mount * (gyro_IMU(i,:)'))';
    mag_IMU_wand(i,:) = (r_imu_mount * (mag_IMU(i,:)'))';
end 


for i = 1 : size(mag_IMU,1)
    q_in = quaternion(ts_imu_quat.Data(i,[4 1 2 3]));
    C_IMU_imu = rotmat(q_in,'frame'); %this rot mat transforms imu to IMU
    mag_fromimutoIMU(i,:) = (C_IMU_imu' * mag_IMU(i,:)')';
    eul_out(i,:) = rad2deg(rotm2eul(C_IMU_imu','ZYX'));

    q_in = quaternion(ts_vicon_quat.Data(i,[4 1 2 3]));
    C_VICON_wand = rotmat(q_in,'frame'); % this rot mat transforms wand to VICON
    mag_fromwandtoVICON(i,:) = (C_VICON_wand' * mag_IMU_wand(i,:)')';
end

real_att_IMUimu = eul_out';

% magnetic field vector in IMU frame
mag_IMU_forfilter = mean(mag_fromimutoIMU);
% magnetic field vectori in VICON frame
mag_VICON_forfilter = mean(mag_fromwandtoVICON);


%% Create a new dataset containing only the measurements of interest:
real_data_IMU = [];

% % IMU measurements in imu frame
real_data_IMU.acc_imu = acc_IMU;
real_data_IMU.gyro_imu = gyro_IMU;
real_data_IMU.mag_imu = mag_IMU;

% IMU measurements in wand frame
real_data_IMU.acc_wand = acc_IMU_wand;
real_data_IMU.gyro_wand = gyro_IMU_wand;
real_data_IMU.mag_wand = mag_IMU_wand;
% Magnetic field measurements in body frame (imu)
real_data_IMU.mag_body = mag_IMU;
% Magnetic field measurements in body frame rotated in IMU frame
real_data_IMU.mag_fromimutoIMU = mag_fromimutoIMU;

real_data_IMU.numSamples = numsamples; % total samples measurements
real_data_IMU.mag_VICON = mag_VICON_forfilter';  % Mean magnetic field in VICON frame
real_data_IMU.mag_IMU = mag_IMU_forfilter';  % Mean magnetic field in IMU frame
real_data_IMU.C_IMU_VICON = C_IMU_VICON;
C_imu_VICON = C_IMU_VICON * C_IMU_imu';
real_data_IMU.C_imu_VICON = C_imu_VICON;

save('real_data.mat','real_data_IMU');

%% Plot
plot_real_data = [];

t = (0:(numsamples-1))/fs;  % time when measurements are provided
plot_real_data.time = t;
plot_real_data.real_att_IMUimu = real_att_IMUimu;
plot_real_data.ts = ts_VICON_wand.Time;
plot_real_data.real_att_VICONwand = (ts_VICON_wand.Data*180/pi)';

save('plot_real_data','plot_real_data')
