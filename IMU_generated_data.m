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

% Body reference system is rotated by -90° around z and 180° around y with
% respect to the navigation reference system (parametrization with
% roll-pitch-yaw ZYX)

% Orientation matrix (composition in local axes):
yaw = -pi/2;
pitch = pi;
roll = 0;
Rz = [  cos(yaw)    -sin(yaw)   0;
        sin(yaw)    cos(yaw)    0;
        0           0           1];     % rotation around z axis
Ry = [  cos(pitch)  0   sin(pitch);
        0           1   0;
        -sin(pitch) 0   cos(pitch)];    % rotation around y axis
Rx = [1 0   0;
        0   cos(roll)   sin(roll);
        0   -sin(roll)  cos(roll)];     % rotation around x axis     
%R = Rz*Ry*Rx;  % composition from left to right: rotation matrix from body frame to navigation frame
R = Rx' * Ry' * Rz';    % composition from right to left: rotation matrix from body frame to navigation frame
initOrientation = inv(R);    % rotation matrix from navigation frame to body frame

traj = kinematicTrajectory('SampleRate',fs,...
    'Velocity',initVel,...
    'Position',initPosition,...
    'Orientation',initOrientation);

[~,orientationNED,~,accNED,angVelNED] = traj(accBody,angVelBody);

%% Generate IMU data without noise
% Create an imuSensor object with ideal accelerometer, magnetometer and gyroscope.
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


%% Generate IMU data with noise
% Create an imuSensor object with realistic accelerometer, magnetometer and gyroscope.
IMU_Noise = imuSensor('accel-gyro-mag','SampleRate',fs);

% IMU_Noise.Accelerometer = accelparams( ...
%     'MeasurementRange',19.62, ...                   % m/s^2
%     'Resolution',0.00059875, ...                    % m/s^2 / LSB
%     'TemperatureScaleFactor',0.008, ...             % % / degree C
%     'ConstantBias',0.4905, ...                      % m/s^2
%     'TemperatureBias',[0.34335 0.34335 0.5886], ... % m/s^2 / degree C
%     'NoiseDensity',0.003924);                       % m/s^2 / Hz^(1/2)
% 
% IMU_Noise.Magnetometer = magparams( ...
%     'MeasurementRange',1200, ...             % uT
%     'Resolution',0.1, ...                    % uT / LSB
%     'TemperatureScaleFactor',0.1, ...        % % / degree C
%     'ConstantBias',1, ...                    % uT
%     'TemperatureBias',[0.8 0.8 2.4], ...     % uT / degree C
%     'NoiseDensity',[0.6 0.6 0.9]/sqrt(100)); % uT / Hz^(1/2)
% 
% IMU_Noise.Gyroscope = gyroparams( ...
%     'MeasurementRange',4.3633, ...
%     'Resolution',0.00013323, ...
%     'AxesMisalignment',2, ...
%     'NoiseDensity',8.7266e-05, ...
%     'TemperatureBias',0.34907, ...
%     'TemperatureScaleFactor',0.02, ...
%     'AccelerationBias',0.00017809, ...
%     'ConstantBias',[0.3491,0.5,0]);

noiseDensity = 5e-04;   % gyro white noise drift
constantBias = [0.01, 0.01, 0.005]; % gyro constant bias 
IMU_Noise.Gyroscope.NoiseDensity = noiseDensity;   % add white noise drift for gyro measurements
IMU_Noise.Gyroscope.ConstantBias = constantBias;    % add a constant bias for gyro measurements (https://www.ericcointernational.com/inertial-measurement-units)

[accelReadingN,gyroReadingN,magReadingN] = IMU_Noise(-accNED,angVelNED,orientationNED);

% Plot the accelerometer readings, gyroscope readings, and magnetometer readings.
t = (0:(totalNumSamples-1))/IMU_Noise.SampleRate;

figure(2)
subplot(3,1,1)
plot(t,accelReadingN)
legend('X-axis','Y-axis','Z-axis')
title('Accelerometer Readings With Noise')
ylabel('Acceleration (m/s^2)')

subplot(3,1,2)
plot(t,gyroReadingN)
legend('X-axis','Y-axis','Z-axis')
title('Gyroscope Readings With Noise')
ylabel('Angular Velocity (rad/s)')

subplot(3,1,3)
plot(t,magReadingN)
legend('X-axis','Y-axis','Z-axis')
title('Magnetometer Readings With Noise')
xlabel('Time (s)')
ylabel('Magnetic Field (uT)')

%% Load data in dataset
log_vars = [];
log_vars.accel = accelReading;
log_vars.gyro = gyroReading;
log_vars.mag = magReading;
log_vars.accelN = accelReadingN;
log_vars.gyroN = gyroReadingN;
log_vars.magN = magReadingN;
log_vars.initOrientation = initOrientation;
log_vars.orientation = orientationNED;
log_vars.frequency = fs;
log_vars.numSamples = totalNumSamples;
log_vars.gyrobias = constantBias;
save('dataset','log_vars');