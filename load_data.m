close all
clear all
clc

% Load dataset
load("all_data_in_ws.mat")

% figure;
% plot(ts_vicon_euler.Time, ts_vicon_euler.Data)
% grid on
% 
% figure;
% plot(ts_IMU_VICON.Time, ts_IMU_VICON.Data*180/pi)
% grid on
% 
figure;
plot(ts_VICON_wand.Time, ts_VICON_wand.Data*180/pi)
legend('Roll','Pitch','Yaw')
title('Real attitude in VICON frame')
xlabel('t [s]')
ylabel('Roll-pitch-yaw angles [deg]')
grid on
xlim([0,size(ts_VICON_wand.Time,1)/10])
newcolors = ["#0B0" "#00F" "#A0F"];
colororder(newcolors)

% % 
% figure;
% plot(ts_VICON_wand.Time, ts_VICON_wand.Data*180/pi);
% hold on
% grid on
% plot(ts_vicon_euler.Time, ts_vicon_euler.Data*180/pi)
% legend('roll (vicon)','pitch (vicon)','yaw (vicon)', 'roll (imu)','pitch (imu)','yaw (imu)');
% hold off

%%

% Extract real data IMU sensor
acc_IMU = ts_imu_acc.Data; % accelerometer measurements
gyro_IMU = ts_imu_gyro.Data;   % gyroscope measurements
mag_IMU = ts_imu_mag.Data; % magnetometer measurements
numsamples = size(acc_IMU,1);   % total number of samples

% Convert data from imu frame to wand frame
% Compute Cimu^wand matrix:
% define IMU mounting:
%  - the IMU X axis is aligned with -Y wand (vicon) axis
%  - the IMU Y axis is aligned with -X wand (vicon) axis 
% thus the rotation from IMU axis to wand_axis is : 
% first Rz(90), then Ry(180) 
roll = 0;
pitch = pi;
% yaw = pi/2;
yaw = pi/2 - deg2rad(10);
Rotx = [1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll)];
Roty = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
Rotz = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1];
r_imu_mount = Rotx*Roty*Rotz; %this is C_imu^wand

acc_IMU_wand = zeros(numsamples,3);
gyro_IMU_wand = zeros(numsamples,3);
mag_IMU_wand = zeros(numsamples,3);

for i = 1 : size(acc_IMU,1)
    acc_IMU_wand(i,:) = (r_imu_mount * (acc_IMU(i,:)'))';
    gyro_IMU_wand(i,:) = (r_imu_mount * (gyro_IMU(i,:)'))';
    mag_IMU_wand(i,:) = (r_imu_mount * (mag_IMU(i,:)'))';
end 


% magnetic field vector in VICON frame
mag_VICON = C_VICON_wand' * mean(mag_IMU_wand(1:7,:))';
% Alternatively it can be computed as
% C_imu_VICON = C_IMU_VICON * C_IMU_imu';
% mag_VICON = C_imu_VICON * mean(mag_IMU(1:7,:))';

% magnetic field vector in IMU frame
mag_IMU_frame = C_IMU_imu' * mean(mag_IMU(1:7,:))';


% Create a new dataset containing only the measurements of interest:
real_data_IMU = [];

% % IMU measurements in imu frame
% real_data_IMU.acc = acc_IMU;
% real_data_IMU.gyro = gyro_IMU;
% real_data_IMU.mag = mag_IMU;

% IMU measurements in wand frame
real_data_IMU.acc = acc_IMU_wand;
real_data_IMU.gyro = gyro_IMU_wand;
real_data_IMU.mag = mag_IMU_wand;

real_data_IMU.numSamples = numsamples; % total samples measurements
real_data_IMU.mag_VICON = mag_VICON;
real_data_IMU.mag_IMU = mag_IMU_frame;
real_data_IMU.C_IMU_VICON = C_IMU_VICON;
C_imu_VICON = C_IMU_VICON * C_IMU_imu';
real_data_IMU.C_imu_VICON = C_imu_VICON;


save('real_data','real_data_IMU');
