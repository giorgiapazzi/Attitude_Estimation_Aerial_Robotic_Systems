clear all
close all
clc

%% Trajectory generation
% The drone makes a vertical take off and moves with small accelerations

% First trajectory: 73 seconds of flight
% The drone moves in vertical for 3 m in 5 s and then rotates 90° in 3 s.
% It then moves forward for 60 seconds with an acceleration of 0.5 m/s^2
% and performs a vertical landing with the same acceleration of the take
% off.
fs = 100;   % Hz
firstLoopSamples = fs*5;    % 500 measures for the first 5 seconds
secondLoopSamples = fs*3;   % 300 measures for the next 3 seconds
thirdLoopSamples = fs*60;   % 6000 measures for the next 60 seconds
fourthLoopSamples = fs*5;   % 500 measures for the final seconds of the trajectory
totalNumSamples = firstLoopSamples + secondLoopSamples + thirdLoopSamples + fourthLoopSamples;

startFirstInterval = 1;
endFirstInterval = firstLoopSamples;
startSecondInterval = endFirstInterval + 1;
endSecondInterval = startSecondInterval + secondLoopSamples - 1;
startThirdInterval = endSecondInterval + 1;
endThirdInterval = startThirdInterval + thirdLoopSamples - 1;
startFourthInterval = endThirdInterval + 1;
endFourthInterval = startFourthInterval + fourthLoopSamples - 1;

% We assume that the reference system for the drone in equal to the ENU
% (east-north-up) at the initial time t = 0.

% Body linear acceleration
accBody = zeros(totalNumSamples,3);
accBody(startFirstInterval:endFirstInterval,3) = 3/5;    % vertical take off
accBody(startThirdInterval:endThirdInterval,2) = 0.5;  % forward movement
accBody(startFourthInterval:endFourthInterval,3) = -3/5;  % vertical landing

% Velocità angolare body
angVelBody = zeros(totalNumSamples,3);
angVelBody(startSecondInterval:endSecondInterval,3) = (pi/2)/3;    % turn left

% Definition of kinematicTrajectory object
initPosition = [0,0,0]; % at time t=0 body RS and navigation RS overlap
initVel = [0,0,0];  % at time t=0 the drone is stationary

% Navigation reference system : NED (north-east-down)
% Body reference system is rotated by -90° around z and 180° around y with
% respect to the navigation reference system

% Orientation with quaternion:
%initOrientation = quaternion([-90,180,0],'eulerd','zyx','frame');   % orientation of body RS wrt navigation RS

% Orientation with matrix:
yaw = -pi/2;
pitch = pi;
roll = 0;
Rz = [  cos(yaw)    -sin(yaw)   0;
        sin(yaw)    cos(yaw)    0;
        0           0           1]; % rotation around z axis
Ry = [  cos(pitch)  0   sin(pitch);
        0           1   0;
        -sin(pitch) 0   cos(pitch)];    % rotation around y axis
R = Rz*Ry;  % composition from left to right
initOrientation = R;

traj = kinematicTrajectory('SampleRate',fs,...
    'Velocity',initVel,...
    'Position',initPosition,...
    'Orientation',initOrientation);

[~,orientationNED,~,accNED,angVelNED] = traj(accBody,angVelBody);

%% Generate IMU data
IMU = imuSensor('accel-gyro-mag','SampleRate',fs);
[accelReading,gyroReading,magReading] = IMU(-accNED,angVelNED,orientationNED);

% Plot the accelerometer readings, gyroscope readings, and magnetometer readings.
t = (0:(totalNumSamples-1))/IMU.SampleRate;

figure(1)
subplot(3,1,1)
plot(t,accelReading)
legend('X-axis','Y-axis','Z-axis')
title('Accelerometer Readings')
ylabel('Acceleration (m/s^2)')

subplot(3,1,2)
plot(t,gyroReading)
legend('X-axis','Y-axis','Z-axis')
title('Gyroscope Readings')
ylabel('Angular Velocity (rad/s)')

subplot(3,1,3)
plot(t,magReading)
legend('X-axis','Y-axis','Z-axis')
title('Magnetometer Readings')
xlabel('Time (s)')
ylabel('Magnetic Field (uT)')

%% Load data in dataset
log_vars = [];
log_vars.accel = accelReading;
log_vars.gyro = gyroReading;
log_vars.mag = magReading;
save('dataset','log_vars');