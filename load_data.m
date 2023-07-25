close all
clear all
clc

% Load dataset
load("all_data_in_ws.mat")

% Extract real data IMU sensor
acc_IMU = ts_imu_acc.Data'; % accelerometer measurements
gyro_IMU = ts_imu_gyro.Data';   % gyroscope measurements
mag_IMU = ts_imu_mag.Data'; % magnetometer measurements

% Create a new dataset containing only the measurements of interest:
% IMU measurements
real_data_IMU = [];
real_data_IMU.acc = acc_IMU;
real_data_IMU.gyro = gyro_IMU;
real_data_IMU.mag = mag_IMU;
real_data_IMU.numSamples = size(acc_IMU,2); % total samples measurements



save('real_data','real_data_IMU');


