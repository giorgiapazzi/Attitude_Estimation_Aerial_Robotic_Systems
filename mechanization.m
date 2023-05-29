clear all
close all
clc

% Load IMU measures
load('dataset')
accel_x = log_vars.accel(:,1);
accel_y = log_vars.accel(:,2);
accel_z = log_vars.accel(:,3);
angvel_x = log_vars.gyro(:,1);
angvel_y = log_vars.gyro(:,2);
angvel_z = log_vars.gyro(:,3);
mag_x = log_vars.mag(:,1);
mag_y = log_vars.mag(:,2);
mag_z = log_vars.mag(:,3);
initOrientation = log_vars.initOrientation;    % local frame orientation wrt navigation frame at time t=0
fs = log_vars.frequency;    % sensors frequency

dt = 1/fs;  % sample time


% A questo punto devo applicare le due equazioni differenziali del filtro
