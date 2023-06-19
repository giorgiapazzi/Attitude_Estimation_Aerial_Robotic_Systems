clear all
close all
clc

% Load IMU measures
load('dataset')
numSamples = log_vars.numSamples;
accel_x = log_vars.accel(:,1);
accel_y = log_vars.accel(:,2);
accel_z = log_vars.accel(:,3);
angvel_x = log_vars.gyro(:,1);
angvel_y = log_vars.gyro(:,2);
angvel_z = log_vars.gyro(:,3);
mag_x = log_vars.mag(:,1);
mag_y = log_vars.mag(:,2);
mag_z = log_vars.mag(:,3);
R0 = log_vars.initOrientation;    % local frame orientation wrt navigation frame at time t=0
Rnb = log_vars.orientation; % navigation frame orientation wrt local frame         
fs = log_vars.frequency;    % sensors frequency

dt = 1/fs;  % sample time


%% Initial attitude values
% Attitude is expressed by roll-pitch-yaw angles wrt navigation frame (in
% order ZYX)
% Compute roll-pitch-yaw angles from orientation matrix. 
% Angles:   roll -> rotation around X
%           pitch -> rotation around Y
%           yaw -> rotation around Z
% Because of pitch angle is 180, we need to use the following expressions:
% yaw = atan2(-R0(2,1),-R0(1,1))
% pitch = atan2(-R0(3,1),-sqrt((R0(3,2))^2,(R0(3,3))^2))
% roll = atan2(-R0(3,2),-R0(3,3))

% Initial orientation of body frame with respect to navigation frame:
yaw_0 = atan2(-R0(2,1),-R0(1,1));
pitch_0 = atan2(-R0(3,1),-sqrt((R0(3,2))^2 + (R0(3,3))^2));
roll_0 = atan2(-R0(3,2),-R0(3,3));


%% Attitude values computed with true mechanization
% Attitude is expressed by yaw-pitch-roll angles wrt navigation frame (in
% order ZYX)
% Compute roll-pitch-yaw angles from orientation matrix. 
true_attitude_angles = zeros(numSamples+1,3);
true_attitude_angles(1,:)= [roll_0,pitch_0,yaw_0];

for i = 1 : (numSamples)
    roll = atan2(-Rnb(3,2,i),-Rnb(3,3,i));
    true_attitude_angles(i,1) = roll;
    pitch = atan2(-Rnb(3,1,i),-sqrt((Rnb(3,2,i))^2 + (Rnb(3,3,i))^2));
    true_attitude_angles(i,2) = pitch;
    yaw = atan2(-Rnb(2,1,i),-Rnb(1,1,i));
    true_attitude_angles(i,3) = yaw;
end


%% Euler angles kinematics
% roll_dot = omega_x + sin(roll)*tan(pitch)*omega_y + cos(roll)*tan(pitch)*omega_z
% pitch_dot = cos(roll)*omega_y - sin(roll)*omega_z
% yaw_dot = sin(roll)/cos(pitch) * omega_y + cos(roll)/cos(pitch) * omega_z

% Discretization with forward Euler method
attitude_angles = zeros(numSamples+1,3);
attitude_angles(1,:)= [roll_0,pitch_0,yaw_0];

for i = 2 : (numSamples+1)
    roll = attitude_angles(i-1,1);
    pitch = attitude_angles(i-1,2);
    yaw = attitude_angles(i-1,3);

    roll = angvel_x(i-1) + sin(roll)*tan(pitch)*angvel_y(i-1) + cos(roll)*tan(pitch)*angvel_z(i-1);
    attitude_angles(i,1) = attitude_angles(i-1,1) + roll*dt;
    pitch = cos(roll)*angvel_y(i-1) - sin(roll)*angvel_z(i-1);
    attitude_angles(i,2) = attitude_angles(i-1,2) + attitude_angles(i-1,2)*dt;
    yaw = sin(roll)/cos(pitch) * angvel_y(i-1) + cos(roll)/cos(pitch) * angvel_z(i-1);
    attitude_angles(i,3) = attitude_angles(i-1,3) + attitude_angles(i-1,3)*dt;

end

