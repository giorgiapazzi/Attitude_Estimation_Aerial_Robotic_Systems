clear all
close all
clc

% Load data
load('dataset')

numSamples = log_vars.numSamples;
m_B = log_vars.mag;
angvel_x = log_vars.gyro(:,1);
angvel_y = log_vars.gyro(:,2);
angvel_z = log_vars.gyro(:,3);
fs = log_vars.frequency;    % sensors frequency

dt = 1/fs;  % sample time

R0 = log_vars.initOrientation;    % rotation matrix from navigation frame to body frame at time t=0
R = log_vars.orientation; % rotation matrix from navigation frame to body frame for t>0 

R = cat(3,R0,R);    % concatenate rotation matrix in one single multidimensional array
g = -9.81;   % gravity acceleration
e3 = [0;0;1];
u_I = e3;
m_I = [27.5550;-2.4169;-16.08049];  % magnetic field in navigation frame 
m_I_norm = m_I / norm(m_I);  % normalized magnetic field in navigation frame

% Initial orientation of body frame with respect to navigation frame:
yaw_0 = atan2(-R0(2,1),-R0(1,1));
pitch_0 = atan2(-R0(3,1),-sqrt((R0(3,2))^2 + (R0(3,3))^2));
roll_0 = atan2(-R0(3,2),-R0(3,3));

% Positive constant gains
k1 = 0.5;
k2 = 0.5;
kb = 0.5;

R_pred = zeros(3,3,numSamples+1);
R_pred(:,:,1) = R0;
b_omega = 0.3;  % initial constant gyro bias
attitude_angles = zeros(numSamples+1,3);
attitude_angles(1,:)= [roll_0,pitch_0,yaw_0];

for i = 1 : (numSamples+1)
    a_B = -g * R(:,:,i)' * e3;   % approximation of accelerometer measurements
    u_B = -a_B./g;
    m_B_norm(:,i) = m_B(i,:)'/norm(m_I);

    u_B_pred = R_pred(:,:,i)' * u_I;
    m_B_norm_pred = R_pred(:,:,i)' * m_I_norm;

    sigma_R = k1 * cross(u_B,u_B_pred) + k2 * cross(m_B_norm,m_B_norm_pred);
    sigma_b = - kb * sigma_R;

    b_omega_dot = sigma_b;
    b_omega(i+1) = b_omega(i) + b_omega_dot*dt;

    roll = attitude_angles(i-1,1);
    pitch = attitude_angles(i-1,2);
    yaw = attitude_angles(i-1,3);

    omega_x = angvel_x(i) - b_omega(1,i) + sigma_R(1,i);
    omega_y = angvel_y(i) - b_omega(2,i) + sigma_R(2,i);
    omega_z = angvel_z(i) - b_omega(3,i) + sigma_R(3,i);


    roll_dot = omega_x -  + sin(roll)*tan(pitch)*omega_y + cos(roll)*tan(pitch)*omega_z;
    attitude_angles(i,1) = attitude_angles(i-1,1) + roll_dot*dt;
    attitude_angles(i,1) = wrapTo2Pi(attitude_angles(i,1));

    pitch_dot = cos(roll)*omega_y - sin(roll)*omega_z;
    attitude_angles(i,2) = attitude_angles(i-1,2) + pitch_dot*dt;
    attitude_angles(i,2) = wrapTo2Pi(attitude_angles(i,2));

    yaw_dot = sin(roll)/cos(pitch) * omega_y + cos(roll)/cos(pitch) * omega_z;
    attitude_angles(i,3) = attitude_angles(i-1,3) + yaw_dot*dt;
    attitude_angles(i,3) = wrapTo2Pi(attitude_angles(i,3));
       
end


