clear all
close all
clc

% Load data
load('real_data.mat')

numSamples = real_data_IMU.numSamples;
m_B = real_data_IMU.mag';    % magnetometer measurements 
acc_B = real_data_IMU.acc';    % accelerometer measurements
angvel_x = real_data_IMU.gyro(:,1); % gyro measurements along x-axis
angvel_y = real_data_IMU.gyro(:,2); % gyro measurements along y-axis
angvel_z = real_data_IMU.gyro(:,3); % gyro measurements along z-axis

fs = 10;    % IMU sensor frequency
dt = 1/fs;

R0 = eye(3,3);
g = 9.81;   % gravity acceleration
e3 = [0;0;1];
u_I = e3;
m_I = [27.5550;-2.4169;-16.08049];  % magnetic field in navigation frame 
norm_mI = norm(m_I);
m_I_norm = m_I / norm_mI;  % normalized magnetic field in navigation frame

%% Initialization
% Initial orientation of body frame with respect to navigation frame:
std_dev_roll = deg2rad(0);  % standard deviation for initial roll angle
std_dev_pitch = deg2rad(0);  % standard deviation for initial pitch angle
std_dev_yaw = deg2rad(0);  % standard deviation for initial yaw angle
% std_dev_roll = deg2rad(1);  % standard deviation for initial roll angle
% std_dev_pitch = deg2rad(3);  % standard deviation for initial pitch angle
% std_dev_yaw = deg2rad(5);  % standard deviation for initial yaw angle
% std_dev_roll = deg2rad(-30);  % standard deviation for initial roll angle
% std_dev_pitch = deg2rad(30);  % standard deviation for initial pitch angle
% std_dev_yaw = deg2rad(90);  % standard deviation for initial yaw angle

roll_0 = atan2(R0(3,2),R0(3,3)) + std_dev_roll * randn(1,1);
pitch_0 = -asin(R0(3,1)) + std_dev_pitch * randn(1,1);
yaw_0 = atan2(R0(2,1),R0(1,1)) + std_dev_yaw * randn(1,1);

fprintf('Initial attitude values: roll: %g  pitch: %g  yaw: %g \n', roll_0,pitch_0,yaw_0);

attitude_angles = zeros(numSamples+1,3);
attitude_angles(1,:)= [roll_0,pitch_0,yaw_0];

R_pred = zeros(3,3,numSamples+1);
R_pred(:,:,1) = R0;

% std_dev_b = 1e-6;   % standard deviation for white noise for initial gyro bias
%b_omega = constantBias';
%b_omega = constantBias' + std_dev_b * randn(3,1);  % initial constant gyro bias with uncertainty
b_omega = [0;0;0];

sigmaR = zeros(3,1);
sigmaB = zeros(3,1);
estimatedAngVel = zeros(3,1);

% Positive constant gains: (k1 = 1, k2 = 1, kb = 0.3)
% k1 and k2 are proportional gains
% kb is integral gain
% the integral gain (kb) governs the dynamics of the gyro-bias estimation
% k1 = 1;
% k2 = 1;
% kb = 0.3;
k1 = 6.371;
k2 = 1.274;
kb = k1/32;
% k1 = 2.9;
% k2 = 1.2;
% kb = 0.1991;

%% Explicit complementary filter
for i = 2 : (numSamples+1)
    % Measured acceleration and magnetic field
    a_B = -acc_B(:,i-1);
    u_B = -a_B./g;
    m_B_norm = m_B(:,i-1)/norm_mI;

    % Estimated acceleration and magnetic field
    u_B_pred = R_pred(:,:,i-1)' * u_I;
    m_B_norm_pred = R_pred(:,:,i-1)' * m_I_norm;

    % Inclination correction
    sigma_R = k1 .* cross(u_B,u_B_pred) + k2 .* cross(m_B_norm,m_B_norm_pred);
    % Constant bias correction
    sigma_b = - kb .* sigma_R;

    % Constant bias
    b_omega_dot = sigma_b;
    b_omega(1,i) = b_omega(1,i-1) + b_omega_dot(1,1)*dt;
    b_omega(2,i) = b_omega(2,i-1) + b_omega_dot(2,1)*dt;
    b_omega(3,i) = b_omega(3,i-1) + b_omega_dot(3,1)*dt;

    % Angles at time t-1
    roll = attitude_angles(i-1,1);
    pitch = attitude_angles(i-1,2);
    yaw = attitude_angles(i-1,3);

    % Angular velocity correction
    omega_x = angvel_x(i-1) - b_omega(1,i) + sigma_R(1);
    omega_y = angvel_y(i-1) - b_omega(2,i) + sigma_R(2);
    omega_z = angvel_z(i-1) - b_omega(3,i) + sigma_R(3);

    % Angles at current time
    roll_dot = omega_x + sin(roll)*tan(pitch)*omega_y + cos(roll)*tan(pitch)*omega_z;
    new_roll = wrapToPi(roll + roll_dot*dt);
    attitude_angles(i,1) = new_roll;
        
    pitch_dot = cos(roll)*omega_y - sin(roll)*omega_z;
    new_pitch = wrapToPi(pitch + pitch_dot*dt);
    attitude_angles(i,2) = new_pitch;

    yaw_dot = sin(roll)/cos(pitch) * omega_y + cos(roll)/cos(pitch) * omega_z;
    new_yaw = wrapToPi(yaw + yaw_dot*dt);
    attitude_angles(i,3) = new_yaw;

    % Compute rotation matrix (Cbn)
    Rz = [  cos(new_yaw)    -sin(new_yaw)   0;
            sin(new_yaw)    cos(new_yaw)    0;
            0               0               1];     % rotation around z axis
    Ry = [  cos(new_pitch)      0   sin(new_pitch);
            0                   1   0;
            -sin(new_pitch)     0   cos(new_pitch)];    % rotation around y axis
    Rx = [  1       0                   0;
            0       cos(new_roll)       -sin(new_roll);
            0       sin(new_roll)      cos(new_roll)];     % rotation around x axis

    R_pred(:,:,i) = Rz*Ry*Rx; % composition from left to right: rotation matrix from body frame to navigation frame

    sigmaR(:,i) = sigma_R;
    sigmaB(:,i) = sigma_b;
    estimatedAngVel(:,i) = [omega_x;omega_y;omega_z];   
end


%% Plot
t = (0:(numSamples))/fs;  % time when measurements are provided

figure(1)
plot(t,rad2deg(attitude_angles(:,1)'))
hold on
plot(t,rad2deg(attitude_angles(:,2)'))
hold on
plot(t,rad2deg(attitude_angles(:,3)'))
legend('Roll','Pitch','Yaw')
title('Attitude estimation')
xlabel('t [s]')
xlim([0,size(t,2)/fs])
ylabel('Roll-pitch-yaw angles [deg]')
grid on

% for i=1 : (numSamples+1)
%     error_roll(i) = true_attitude_angles(i,1) - attitude_angles(i,1);
%     error_pitch(i) = true_attitude_angles(i,2) - attitude_angles(i,2);
%     error_yaw(i) = true_attitude_angles(i,3) - attitude_angles(i,3);
% end
% 
% figure(2)
% plot(t,rad2deg(error_roll))
% hold on
% plot(t,rad2deg(error_pitch))
% hold on
% plot(t,rad2deg(error_yaw))
% legend('Roll Error','Pitch Error','Yaw Error')
% title('Attitude estimation error')
% xlabel('t [s]')
% xlim([0,size(t,2)/fs])
% ylabel('Roll-pitch-yaw angles error [deg]')
% grid on
