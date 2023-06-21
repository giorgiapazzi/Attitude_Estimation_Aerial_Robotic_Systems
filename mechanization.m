clear all
close all
clc

% Load data
load('dataset')

numSamples = log_vars.numSamples;
R0 = log_vars.initOrientation;    % rotation matrix from navigation frame to body frame at time t=0
R = log_vars.orientation; % rotation matrix from navigation frame to body frame for t>0         
fs = log_vars.frequency;    % sensors frequency

dt = 1/fs;  % sample time

% Load IMU measures without noise
accel_x = log_vars.accel(:,1);
accel_y = log_vars.accel(:,2);
accel_z = log_vars.accel(:,3);
angvel_x = log_vars.gyro(:,1);
angvel_y = log_vars.gyro(:,2);
angvel_z = log_vars.gyro(:,3);
mag_x = log_vars.mag(:,1);
mag_y = log_vars.mag(:,2);
mag_z = log_vars.mag(:,3);

% Load IMU measures with noise
accel_x_N = log_vars.accelN(:,1);
accel_y_N = log_vars.accelN(:,2);
accel_z_N = log_vars.accelN(:,3);
angvel_x_N = log_vars.gyroN(:,1);
angvel_y_N = log_vars.gyroN(:,2);
angvel_z_N = log_vars.gyroN(:,3);
mag_x_N = log_vars.magN(:,1);
mag_y_N = log_vars.magN(:,2);
mag_z_N = log_vars.magN(:,3);


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
    % Compute roll angle
    roll = wrapTo2Pi(atan2(-R(3,2,i),-R(3,3,i)));
    roll = round(roll,2);
    % Rotation of -pi or +pi is equivalent: control to prevent oscillations
    % between -pi and +pi
    if roll == 3.14
        if true_attitude_angles(i,1) == -3.14
            roll = -3.14;
        end
    elseif roll == -3.14
        if true_attitude_angles(i,1) == 3.14
            roll = 3.14;
        end
    end
    true_attitude_angles(i+1,1) = roll;

    % Compute pitch angle
    pitch = wrapTo2Pi(atan2(-R(3,1,i),-sqrt((R(3,2,i))^2 + (R(3,3,i))^2)));
    pitch = round(pitch,2);
    % Rotation of -pi or +pi is equivalent: control to prevent oscillations
    % between -pi and +pi
    if pitch == 3.14
        if true_attitude_angles(i,2) == -3.14
            pitch = -3.14;
        end
    elseif pitch == -3.14
        if true_attitude_angles(i,2) == 3.14
            pitch = 3.14;
        end
    end
    true_attitude_angles(i+1,2) = pitch;

    % Compute yaw angle
    yaw = wrapTo2Pi(atan2(-R(2,1,i),-R(1,1,i)));
    yaw = round(yaw,2);
    % Rotation of -pi or +pi is equivalent: control to prevent oscillations
    % between -pi and +pi
    if yaw == 3.14
        if true_attitude_angles(i,3) == -3.14
            yaw = -3.14;
        end
    elseif yaw == -3.14
        if true_attitude_angles(i,3) == 3.14
            yaw = 3.14;
        end
    end
    true_attitude_angles(i+1,3) = yaw;
end

%% Plot
t = (0:(numSamples))/fs;  % time when measurements are provided

figure(1)
plot(t,true_attitude_angles(:,1)')
hold on
plot(t,true_attitude_angles(:,2)')
hold on
plot(t,true_attitude_angles(:,3)')
legend('Roll','Pitch','Yaw')
title('Attitude estimation')
xlabel('t [s]')
ylabel('Roll-pitch-yaw angles [rad]')


%% Euler angles kinematics wihout noise
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

    roll_dot = angvel_x(i-1) + sin(roll)*tan(pitch)*angvel_y(i-1) + cos(roll)*tan(pitch)*angvel_z(i-1);
    attitude_angles(i,1) = attitude_angles(i-1,1) + roll_dot*dt;
    attitude_angles(i,1) = wrapTo2Pi(attitude_angles(i,1));

    pitch_dot = cos(roll)*angvel_y(i-1) - sin(roll)*angvel_z(i-1);
    attitude_angles(i,2) = attitude_angles(i-1,2) + pitch_dot*dt;
    attitude_angles(i,2) = wrapTo2Pi(attitude_angles(i,2));

    yaw_dot = sin(roll)/cos(pitch) * angvel_y(i-1) + cos(roll)/cos(pitch) * angvel_z(i-1);
    attitude_angles(i,3) = attitude_angles(i-1,3) + yaw_dot*dt;
    attitude_angles(i,3) = wrapTo2Pi(attitude_angles(i,3));
end


%% Plot
t = (0:(numSamples))/fs;  % time when measurements are provided

figure(2)
plot(t,attitude_angles(:,1)')
hold on
plot(t,attitude_angles(:,2)')
hold on
plot(t,attitude_angles(:,3)')
legend('Roll','Pitch','Yaw')
title('Attitude estimation')
xlabel('t [s]')
ylabel('Roll-pitch-yaw angles [rad]')

%% Euler angles kinematics with noise
% roll_dot = omega_x + sin(roll)*tan(pitch)*omega_y + cos(roll)*tan(pitch)*omega_z
% pitch_dot = cos(roll)*omega_y - sin(roll)*omega_z
% yaw_dot = sin(roll)/cos(pitch) * omega_y + cos(roll)/cos(pitch) * omega_z

% Discretization with forward Euler method
attitude_angles_N = zeros(numSamples+1,3);
attitude_angles_N(1,:)= [roll_0,pitch_0,yaw_0];   % we assume to know exactly the initial position of the drone

for i = 2 : (numSamples+1)
    roll = attitude_angles_N(i-1,1);
    pitch = attitude_angles_N(i-1,2);
    yaw = attitude_angles_N(i-1,3);

    roll_dot = angvel_x_N(i-1) + sin(roll)*tan(pitch)*angvel_y_N(i-1) + cos(roll)*tan(pitch)*angvel_z_N(i-1);
    attitude_angles_N(i,1) = attitude_angles_N(i-1,1) + roll_dot*dt;
    attitude_angles_N(i,1) = wrapTo2Pi(attitude_angles_N(i,1));

    pitch_dot = cos(roll)*angvel_y_N(i-1) - sin(roll)*angvel_z_N(i-1);
    attitude_angles_N(i,2) = attitude_angles_N(i-1,2) + pitch_dot*dt;
    attitude_angles_N(i,2) = wrapTo2Pi(attitude_angles_N(i,2));

    yaw_dot = sin(roll)/cos(pitch) * angvel_y_N(i-1) + cos(roll)/cos(pitch) * angvel_z_N(i-1);
    attitude_angles_N(i,3) = attitude_angles_N(i-1,3) + yaw_dot*dt;
    attitude_angles_N(i,3) = wrapTo2Pi(attitude_angles_N(i,3));
end

%% Plot
t = (0:(numSamples))/fs;  % time when measurements are provided

figure(2)
plot(t,attitude_angles_N(:,1)')
hold on
plot(t,attitude_angles_N(:,2)')
hold on
plot(t,attitude_angles_N(:,3)')
legend('Roll','Pitch','Yaw')
title('Attitude estimation')
xlabel('t [s]')
ylabel('Roll-pitch-yaw angles [rad]')


%% Error plot
for i=1 : (numSamples+1)
    error_roll(i) = true_attitude_angles(i,1) - attitude_angles_N(i,1);
    error_pitch(i) = true_attitude_angles(i,2) - attitude_angles_N(i,2);
    error_yaw(i) = true_attitude_angles(i,3) - attitude_angles_N(i,3);
end

t = (0:(numSamples))/fs;  % time when measurements are provided

figure(3)
plot(t,error_roll)
hold on
plot(t,error_pitch)
hold on
plot(t,error_yaw)
legend('Roll Error','Pitch Error','Yaw Error')
title('Attitude estimation error')
xlabel('t [s]')
ylabel('Roll-pitch-yaw angles error [rad]')
