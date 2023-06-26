clear all
close all
clc

% Load data
load('dataset')

numSamples = log_vars.numSamples;
m_B = log_vars.mag';    % magnetometer measurements 
angvel_x = log_vars.gyroN(:,1); % gyro measurements along x-axis
angvel_y = log_vars.gyroN(:,2); % gyro measurements along y-axis
angvel_z = log_vars.gyroN(:,3); % gyro measurements along z-axis
acc = log_vars.accelN'; % accelerometer measurements
fs = log_vars.frequency;    % sensors frequency
constantBias = log_vars.gyrobias;   % true constant gyro bias
true_attitude_angles = log_vars.trueAttitudeAngles; % attitude angles computed with true machanization

dt = 1/fs;  % sample time

R0 = log_vars.initOrientation;    % rotation matrix from navigation frame to body frame at time t=0
R = log_vars.orientation;   % rotation matrix from navigation frame to body frame for t>0 

std_dev_R = 1e-03;  % standard deviation for initial rotation matrix
R0 = R0 + std_dev_R * ones(3,3) * randn(3,1);   % initial rotation matrix with uncertainty
R = cat(3,R0,R);    % concatenate rotation matrix in one single multidimensional array
g = 9.81;   % gravity acceleration
e3 = [0;0;1];
u_I = e3;
m_I = [27.5550;-2.4169;-16.08049];  % magnetic field in navigation frame 
m_I_norm = m_I / norm(m_I);  % normalized magnetic field in navigation frame

%% Initialization
% Initial orientation of body frame with respect to navigation frame:
yaw_0 = atan2(-R0(2,1),-R0(1,1));
pitch_0 = atan2(-R0(3,1),-sqrt((R0(3,2))^2 + (R0(3,3))^2));
roll_0 = atan2(-R0(3,2),-R0(3,3));

R_pred = zeros(3,3,numSamples+1);
R_pred(:,:,1) = R0;

std_dev_b = 1e-3;   % standard deviation for white noise for initial gyro bias
b_omega = [0.01; 0.01; 0.005] + std_dev_b * randn(3,1);  % initial constant gyro bias with uncertainty

attitude_angles = zeros(numSamples+1,3);
attitude_angles(1,:)= [roll_0,pitch_0,yaw_0];

% Positive constant gains
k1 = 0.18;  % 0.18
k2 = 0.18;  % 0.18
kb = 0.15;  % 0.15

%% Explicit complementary filter
for i = 1 : (numSamples)
    %a_B = R(:,:,i)' * (g .* e3);   % approximation of accelerometer measurements
    a_B = acc(:,i);
    u_B = -a_B./g;
    m_B_norm = m_B(:,i)/norm(m_I);

    u_B_pred = R_pred(:,:,i)' * u_I;
    m_B_norm_pred = R_pred(:,:,i)' * m_I_norm;

    sigma_R = k1 * cross(u_B,u_B_pred) + k2 * cross(m_B_norm,m_B_norm_pred);
    sigma_b = - kb * sigma_R;

    b_omega_dot = sigma_b;
    b_omega(:,i+1) = b_omega(:,i) + b_omega_dot*dt;
    disp(b_omega(:,i+1))

    roll = attitude_angles(i,1);
    pitch = attitude_angles(i,2);
    yaw = attitude_angles(i,3);

    omega_x = angvel_x(i) - b_omega(1,i) + sigma_R(1);
    omega_y = angvel_y(i) - b_omega(2,i) + sigma_R(2);
    omega_z = angvel_z(i) - b_omega(3,i) + sigma_R(3);

    roll_dot = omega_x + sin(roll)*tan(pitch)*omega_y + cos(roll)*tan(pitch)*omega_z;
    attitude_angles(i+1,1) = attitude_angles(i,1) + roll_dot*dt;
    attitude_angles(i+1,1) = wrapTo2Pi(attitude_angles(i+1,1));

    pitch_dot = cos(roll)*omega_y - sin(roll)*omega_z;
    attitude_angles(i+1,2) = attitude_angles(i,2) + pitch_dot*dt;
    attitude_angles(i+1,2) = wrapTo2Pi(attitude_angles(i+1,2));

    yaw_dot = sin(roll)/cos(pitch) * omega_y + cos(roll)/cos(pitch) * omega_z;
    attitude_angles(i+1,3) = attitude_angles(i,3) + yaw_dot*dt;
    attitude_angles(i+1,3) = wrapTo2Pi(attitude_angles(i+1,3));

    Rz = [  cos(attitude_angles(i+1,3))     -sin(attitude_angles(i+1,3))    0;
            sin(attitude_angles(i+1,3))     cos(attitude_angles(i+1,3))     0;
            0                               0                               1];     % rotation around z axis
    Ry = [  cos(attitude_angles(i+1,2))     0   sin(attitude_angles(i+1,2));
            0                               1   0;
            -sin(attitude_angles(i+1,2))    0   cos(attitude_angles(i+1,2))];    % rotation around y axis
    Rx = [  1       0                               0;
            0       cos(attitude_angles(i+1,1))     sin(attitude_angles(i+1,1));
            0       -sin(attitude_angles(i+1,1))    cos(attitude_angles(i+1,1))];     % rotation around x axis     
    R_pred(:,:,i+1) = Rz*Ry*Rx; % composition from left to right: rotation matrix from body frame to navigation frame
       
end


%% Plot
t = (0:(numSamples))/fs;  % time when measurements are provided

figure(1)
plot(t,attitude_angles(:,1)')
hold on
plot(t,attitude_angles(:,2)')
hold on
plot(t,attitude_angles(:,3)')
legend('Roll','Pitch','Yaw')
title('Attitude estimation')
xlabel('t [s]')
ylabel('Roll-pitch-yaw angles [rad]')

for i=1 : (numSamples+1)
    error_roll(i) = true_attitude_angles(i,1) - attitude_angles(i,1);
    error_pitch(i) = true_attitude_angles(i,2) - attitude_angles(i,2);
    error_yaw(i) = true_attitude_angles(i,3) - attitude_angles(i,3);
end

figure(2)
plot(t,error_roll)
hold on
plot(t,error_pitch)
hold on
plot(t,error_yaw)
legend('Roll Error','Pitch Error','Yaw Error')
title('Attitude estimation error')
xlabel('t [s]')
ylabel('Roll-pitch-yaw angles error [rad]')

figure(3)
plot(t,b_omega(1,:))
hold on
plot(t,b_omega(2,:))
hold on
plot(t,b_omega(3,:))
legend('x-bias','y-bias','z-bias')
title('Gyro bias estimation')
xlabel('t [s]')
ylabel('Gyro bias [rad]')

figure(4)
plot(t,(b_omega(1,:)-constantBias(1)))
hold on
plot(t,(b_omega(2,:)-constantBias(2)))
hold on
plot(t,(b_omega(3,:)-constantBias(3)))
legend('x-bias error','y-bias error','z-bias error')
title('Gyro bias estimation error')
xlabel('t [s]')
ylabel('Gyro bias error [rad]')
