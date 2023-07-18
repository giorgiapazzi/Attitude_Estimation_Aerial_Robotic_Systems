clear all
close all
clc

% Load data
load('dataset')

numSamples = log_vars.numSamples;
trajectory = log_vars.trajectory;
frame = log_vars.frame;
m_B = log_vars.mag';    % magnetometer measurements 
angvel_x = log_vars.gyroN(:,1); % gyro measurements along x-axis
angvel_y = log_vars.gyroN(:,2); % gyro measurements along y-axis
angvel_z = log_vars.gyroN(:,3); % gyro measurements along z-axis
acc = log_vars.accelN'; % accelerometer measurements
fs = log_vars.frequency;    % sensors frequency
constantBias = log_vars.gyrobias;   % true constant gyro bias
true_attitude_angles = log_vars.trueAttitudeAngles; % attitude angles computed with true machanization

dt = 1/fs;  % sample time

%R0 = log_vars.initOrientation;    % rotation matrix from body frame to navigation frame at time t=0
R0 = eye(3,3);
R = log_vars.orientation;   % rotation matrix from body frame to navigation frame for t>0 

std_dev_R = 1e-03;  % standard deviation for initial rotation matrix
%R0 = R0 + std_dev_R * ones(3,3) * randn(3,1);   % initial rotation matrix with uncertainty
R = cat(3,R0,R);    % concatenate rotation matrices in one single multidimensional array
g = 9.81;   % gravity acceleration
e3 = [0;0;1];
u_I = e3;
m_I = [27.5550;-2.4169;-16.08049];  % magnetic field in navigation frame 
norm_mI = norm(m_I);
m_I_norm = m_I / norm_mI;  % normalized magnetic field in navigation frame

%% Initialization
% Initial orientation of body frame with respect to navigation frame:
% roll_0 = atan2(-R0(3,2),-R0(3,3));
% pitch_0 = atan2(-R0(3,1),-sqrt((R0(3,2))^2 + (R0(3,3))^2));
% yaw_0 = atan2(-R0(2,1),-R0(1,1));
roll_0 = atan2(R0(3,2),R0(3,3));
pitch_0 = -asin(R0(3,1));
yaw_0 = atan2(R0(2,1),R0(1,1));

attitude_angles = zeros(numSamples+1,3);
attitude_angles(1,:)= [roll_0,pitch_0,yaw_0];

R_pred = zeros(3,3,numSamples+1);
R_pred(:,:,1) = R0;

std_dev_b = 1e-6;   % standard deviation for white noise for initial gyro bias
%b_omega = constantBias';
%b_omega = constantBias' + std_dev_b * randn(3,1);  % initial constant gyro bias with uncertainty
b_omega = [0;0;0];

std_dev_acc = 1e-5; % standard deviation fro white noise for acceleration

sigmaR = zeros(3,1);
sigmaB = zeros(3,1);
estimatedAngVel = zeros(3,1);

% Positive constant gains:
k1 = 1;
k2 = 1;
kb = 0.3;
% if trajectory==1 && frame == 2
%     k1 = 1;  
%     k2 = 1;  
%     kb = 0.008;
% end
% if trajectory==3 && frame==1
%     k1 = 0.9;
%     k2 = 0.9;
%     kb = 0.01;
% end

fprintf('Selected trajectory: %d    Selected frame: %d      k1 = %f     k2 = %f     kb = %f \n',trajectory,frame,k1,k2,kb)


%% Explicit complementary filter
for i = 2 : (numSamples+1)
    a_B = -R(:,:,i-1)' * (g .* e3);  % + (std_dev_acc * randn(3,1));   % approximation of accelerometer measurements
    %a_B = acc(:,i-1);
    u_B = -a_B./g;
    m_B_norm = m_B(:,i-1)/norm_mI;

    u_B_pred = R_pred(:,:,i-1)' * u_I;
    m_B_norm_pred = R_pred(:,:,i-1)' * m_I_norm;

    sigma_R = k1 .* cross(u_B,u_B_pred) + k2 .* cross(m_B_norm,m_B_norm_pred);
    sigma_b = - kb .* sigma_R;

    b_omega_dot = sigma_b;
    b_omega(1,i) = b_omega(1,i-1) + b_omega_dot(1,1)*dt;
    b_omega(2,i) = b_omega(2,i-1) + b_omega_dot(2,1)*dt;
    b_omega(3,i) = b_omega(3,i-1) + b_omega_dot(3,1)*dt;
    %disp(b_omega(:,i))

    roll = attitude_angles(i-1,1);
    pitch = attitude_angles(i-1,2);
    yaw = attitude_angles(i-1,3);

    omega_x = angvel_x(i-1) - b_omega(1,i) + sigma_R(1);
    omega_y = angvel_y(i-1) - b_omega(2,i) + sigma_R(2);
    omega_z = angvel_z(i-1) - b_omega(3,i) + sigma_R(3);

    roll_dot = omega_x + sin(roll)*tan(pitch)*omega_y + cos(roll)*tan(pitch)*omega_z;
    new_roll = wrapToPi(roll + roll_dot*dt);
    attitude_angles(i,1) = new_roll;
        
    pitch_dot = cos(roll)*omega_y - sin(roll)*omega_z;
    new_pitch = wrapToPi(pitch + pitch_dot*dt);
    attitude_angles(i,2) = new_pitch;

    yaw_dot = sin(roll)/cos(pitch) * omega_y + cos(roll)/cos(pitch) * omega_z;
    new_yaw = wrapToPi(yaw + yaw_dot*dt);
    attitude_angles(i,3) = new_yaw;

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
    %R_pred(:,:,i) = Rx' * Ry' * Rz';

    sigmaR(:,i) = sigma_R;
    sigmaB(:,i) = sigma_b;
    estimatedAngVel(:,i) = [omega_x;omega_y;omega_z];
        
end


%% Plot
t = (0:(numSamples))/fs;  % time when measurements are provided

% Delete oscillation between -pi and pi
old_roll = attitude_angles(1,1);
old_pitch = attitude_angles(1,2);
old_yaw = attitude_angles(1,3);
attitude_angles_plot = attitude_angles; 
% Compute estimation error
error_roll(1) = true_attitude_angles(1,1) - old_roll;
error_pitch(1) = true_attitude_angles(1,2) - old_pitch;
error_yaw(1) = true_attitude_angles(1,3) - old_yaw;
for i = 2 : (numSamples+1)
    new_roll = attitude_angles(i,1);
    if round(new_roll,2) == -round(roll,2)
        new_roll = -new_roll;
        attitude_angles_plot(i,1) = new_roll;
    end
    error_roll(i) = true_attitude_angles(i,1) - new_roll;

    new_pitch = attitude_angles(i,2);
    if round(new_pitch,2) == -round(pitch,2)
        new_pitch = -new_pitch;
        attitude_angles_plot(i,2) = new_pitch;
    end
    error_pitch(i) = true_attitude_angles(i,2) - new_pitch;

    new_yaw = attitude_angles(i,3);
    if round(new_yaw,2) == -round(yaw,2)
        new_yaw = -new_yaw;
        attitude_angles_plot(i,3) = new_yaw;
    end
    error_yaw(i) = true_attitude_angles(i,3) - new_yaw;
end


figure(1)
plot(t,attitude_angles(:,1)')
hold on
plot(t,attitude_angles(:,2)')
hold on
plot(t,attitude_angles(:,3)')
legend('Roll','Pitch','Yaw')
title('Attitude estimation')
xlabel('t [s]')
xlim([0,size(t,2)/fs])
ylabel('Roll-pitch-yaw angles [rad]')
grid on


figure(2)
plot(t,attitude_angles_plot(:,1)')
hold on
plot(t,attitude_angles_plot(:,2)')
hold on
plot(t,attitude_angles_plot(:,3)')
legend('Roll','Pitch','Yaw')
title('Attitude estimation without oscillation between [-pi;pi]')
xlabel('t [s]')
xlim([0,size(t,2)/fs])
ylabel('Roll-pitch-yaw angles [rad]')
grid on

% for i=1 : (numSamples+1)
%     error_roll(i) = true_attitude_angles(i,1) - attitude_angles(i,1);
%     error_pitch(i) = true_attitude_angles(i,2) - attitude_angles(i,2);
%     error_yaw(i) = true_attitude_angles(i,3) - attitude_angles(i,3);
% end

figure(3)
plot(t,error_roll)
hold on
plot(t,error_pitch)
hold on
plot(t,error_yaw)
legend('Roll Error','Pitch Error','Yaw Error')
title('Attitude estimation error')
xlabel('t [s]')
xlim([0,size(t,2)/fs])
ylabel('Roll-pitch-yaw angles error [rad]')
grid on

figure(4)
plot(t,b_omega(1,:))
hold on
plot(t,b_omega(2,:))
hold on
plot(t,b_omega(3,:))
legend('x-bias','y-bias','z-bias')
title('Gyro bias estimation')
xlabel('t [s]')
xlim([0,size(t,2)/fs])
ylabel('Gyro bias [rad]')
grid on

gyro_bias_error(1,:) = b_omega(1,:)-constantBias(1);
gyro_bias_error(2,:) = b_omega(2,:)-constantBias(2);
gyro_bias_error(3,:) = b_omega(3,:)-constantBias(3);

figure(5)
plot(t,gyro_bias_error(1,:))
hold on
plot(t,gyro_bias_error(2,:))
hold on
plot(t,gyro_bias_error(3,:))
legend('x-bias error','y-bias error','z-bias error')
title('Gyro bias estimation error')
xlabel('t [s]')
xlim([0,size(t,2)/fs])
ylabel('Gyro bias error [rad]')
grid on

figure(6)
for i = 1 : size(b_omega,2)
    correction(:,i) = -b_omega(:,i)+sigmaR(:,i);
end
plot(t,correction(1,:))
hold on
plot(t,correction(2,:))
hold on
plot(t,correction(3,:))
legend('x-bias correction','y-bias correction','z-bias correction')
title('Angular velocity bias correction')
xlabel('t [s]')
xlim([0,size(t,2)/fs])
ylabel('Gyro bias [rad]')
grid on
