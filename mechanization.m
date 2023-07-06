clear all
close all
clc

% Load data
load('dataset')

numSamples = log_vars.numSamples;
R0 = log_vars.initOrientation;    % rotation matrix from body frame to navigation frame at time t=0
R = log_vars.orientation; % rotation matrix from body frame to navigation frame for t>0         
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
% roll = atan2(-R0(3,2),-R0(3,3))
% pitch = atan2(-R0(3,1),-sqrt((R0(3,2))^2,(R0(3,3))^2))
% yaw = atan2(-R0(2,1),-R0(1,1))
% Alternatively we can use the article's equations:
% roll = atan2(R(3,2),R(3,3))
% pitch = -asin(R(3,1))
% yaw = atan2(R(2,1),R(1,1))

% Initial orientation of body frame with respect to navigation frame:
% roll_0 = atan2(-R0(3,2),-R0(3,3));
% pitch_0 = atan2(-R0(3,1),-sqrt((R0(3,2))^2 + (R0(3,3))^2));
% yaw_0 = atan2(-R0(2,1),-R0(1,1));

roll_0 = atan2(R0(3,2),R0(3,3));
pitch_0 = -asin(R0(3,1));
yaw_0 = atan2(R0(2,1),R0(1,1));

fprintf('Initial attitude angles: roll %f   pitch %f    yaw %f\n', roll_0, pitch_0, yaw_0);


%% Attitude values computed with true mechanization
% Attitude is expressed by yaw-pitch-roll angles wrt navigation frame (in
% order ZYX)
% Compute roll-pitch-yaw angles from orientation matrix. 
true_attitude_angles = zeros(numSamples+1,3);
true_attitude_angles(1,:)= [roll_0,pitch_0,yaw_0];

roll = roll_0;
pitch = pitch_0;
yaw = yaw_0;
for i = 1 : (numSamples)
    % Compute roll angle
    new_roll = wrapToPi(atan2(R(3,2,i),R(3,3,i)));
    % Rotation of -pi or +pi is equivalent: control to prevent oscillations
    % between -pi and +pi
%     if round(new_roll,2) == 3.14
%         if round(roll,2) == -3.14
%             new_roll = -new_roll;
%         end
%     elseif round(new_roll,2) == -3.14
%         if round(roll,2) == 3.14
%             new_roll = -new_roll;
%         end
%     end
    if round(new_roll,2) == -round(roll,2)
        new_roll = - new_roll;
    end
    true_attitude_angles(i+1,1) = new_roll;
    roll = new_roll;

    % Compute pitch angle
    new_pitch = wrapToPi(-asin(R(3,1,i)));
    % Rotation of -pi or +pi is equivalent: control to prevent oscillations
    % between -pi and +pi
%     if round(new_pitch,2) == 3.14
%         if round(pitch,2) == -3.14
%             new_pitch = -new_pitch;
%         end
%     elseif round(new_pitch,2) == -3.14
%         if round(pitch,2) == 3.14
%             new_pitch = -new_pitch;
%         end
%     end
    if round(new_pitch,2) == -round(pitch,2)
        new_pitch = -new_pitch;
    end
    true_attitude_angles(i+1,2) = new_pitch;
    pitch = new_pitch;

    % Compute yaw angle
    new_yaw = wrapTo2Pi(atan2(R(2,1,i),R(1,1,i)));
    % Rotation of -pi or +pi is equivalent: control to prevent oscillations
    % between -pi and +pi
    if round(new_yaw,2) == 6.28
        if round(yaw) == 0
            new_yaw = new_yaw - 2*pi;
        end
    elseif round(new_yaw) == 0
        if round(yaw,2) == 6.28
            new_yaw = new_yaw + 2*pi;
        end
    end
    true_attitude_angles(i+1,3) = new_yaw;
    yaw = new_yaw;
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
    new_roll = wrapToPi(attitude_angles(i-1,1) + roll_dot*dt);
%     if round(new_roll,2) == 3.14
%         if round(roll,2) == -3.14
%             new_roll = -new_roll;
%         end
%     elseif round(new_roll,2) == -3.14
%         if round(roll,2) == 3.14
%             new_roll = -new_roll;
%         end
%     end
    if round(new_roll,2) == -round(roll,2)
        new_roll = - new_roll;
    end
    attitude_angles(i,1) = new_roll;


    pitch_dot = cos(roll)*angvel_y(i-1) - sin(roll)*angvel_z(i-1);
    new_pitch = wrapToPi(attitude_angles(i-1,2) + pitch_dot*dt);
%     if round(new_pitch,2) == 3.14
%         if round(pitch,2) == -3.14
%             new_pitch = -new_pitch;
%         end
%     elseif round(new_pitch,2) == -3.14
%         if round(pitch,2) == 3.14
%             new_pitch = -new_pitch;
%         end
%     end
    if round(new_pitch,2) == -round(pitch,2)
        new_pitch = -new_pitch;
    end    
    attitude_angles(i,2) = new_pitch;

    yaw_dot = sin(roll)/cos(pitch) * angvel_y(i-1) + cos(roll)/cos(pitch) * angvel_z(i-1);
    new_yaw = wrapTo2Pi(attitude_angles(i-1,3) + yaw_dot*dt);
    if round(new_yaw,2) == 6.28
        if round(yaw) == 0
            new_yaw = new_yaw - 2*pi;
        end
    elseif round(new_yaw) == 0
        if round(yaw,2) == 6.28
            new_yaw = new_yaw + 2*pi;
        end
    end
    attitude_angles(i,3) = new_yaw;
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
    new_roll = wrapToPi(attitude_angles_N(i-1,1) + roll_dot*dt);
    if round(new_roll,2) == -round(roll,2)
        new_roll = - new_roll;
    end
    attitude_angles_N(i,1) = new_roll;

    pitch_dot = cos(roll)*angvel_y_N(i-1) - sin(roll)*angvel_z_N(i-1);
    new_pitch = wrapToPi(attitude_angles_N(i-1,2) + pitch_dot*dt);
    if round(new_pitch,2) == -round(pitch,2)
        new_pitch = -new_pitch;
    end
    attitude_angles_N(i,2) = new_pitch;

    yaw_dot = sin(roll)/cos(pitch) * angvel_y_N(i-1) + cos(roll)/cos(pitch) * angvel_z_N(i-1);
    new_yaw = wrapTo2Pi(attitude_angles_N(i-1,3) + yaw_dot*dt);
    if round(new_yaw,2) == 6.28
        if round(yaw) == 0
            new_yaw = new_yaw - 2*pi;
        end
    elseif round(new_yaw) == 0
        if round(yaw,2) == 6.28
            new_yaw = new_yaw + 2*pi;
        end
    end
    attitude_angles_N(i,3) = new_yaw;
end

%% Plot
t = (0:(numSamples))/fs;  % time when measurements are provided

figure(3)
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

figure(4)
plot(t,error_roll)
hold on
plot(t,error_pitch)
hold on
plot(t,error_yaw)
legend('Roll Error','Pitch Error','Yaw Error')
title('Attitude estimation error')
xlabel('t [s]')
ylabel('Roll-pitch-yaw angles error [rad]')

figure(5)
plot(t,error_roll)
title('Roll estimation error')
xlabel('t [s]')
ylabel('Roll angle error [rad]')

figure(6)
plot(t,error_pitch)
title('Pitch estimation error')
xlabel('t [s]')
ylabel('Pitch angle error [rad]')

figure(7)
plot(t,error_yaw)
title('Yaw estimation error')
xlabel('t [s]')
ylabel('Yaw angle error [rad]')


%% Load data in dataset
log_vars.trueAttitudeAngles = true_attitude_angles;
save('dataset','log_vars');