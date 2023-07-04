clear all
close all
clc

%% Trajectory generation
% The drone makes a vertical take off and moves with small accelerations
fs = 100;   % Hz
selectTrajectory = 2;   % write the number of the trajectory to use

if selectTrajectory == 1
    disp('First trajectory selected')
    % First trajectory: 73 seconds of flight
    % The drone moves in vertical in 5 s and then rotates 90° in 3 s.
    % It then moves forward for 60 seconds with an acceleration of 0.5 m/s^2
    % and performs a vertical landing with the same acceleration of the take
    % off.
    firstLoopSamples = fs*5;    % 500 measurements for the first 5 seconds
    secondLoopSamples = fs*3;   % 300 measurements for the next 3 seconds
    thirdLoopSamples = fs*60;   % 6000 measurements for the next 60 seconds
    fourthLoopSamples = fs*5;   % 500 measurements for the final seconds of the trajectory
    totalNumSamples = firstLoopSamples + secondLoopSamples + thirdLoopSamples + fourthLoopSamples;
    
    % First interval and second interval: take off acceleration and deceleration
    startFirstInterval = 1;
    endFirstInterval = fix((startFirstInterval + firstLoopSamples) / 2);
    startSecondInterval = endFirstInterval + 1;
    endSecondInterval = startSecondInterval + fix((startFirstInterval + firstLoopSamples) / 2) - 1;
    % Third interval: turn left
    startThirdInterval = endSecondInterval + 1;
    endThirdInterval = startThirdInterval + secondLoopSamples - 1;
    % Fourth and fifth interval: forward acceleration
    startFourthInterval = endThirdInterval + 1;
    endFifthInterval = startFourthInterval + thirdLoopSamples - 1;
    endFourthInterval = fix((startFourthInterval + endFifthInterval) / 2);
    startFifthInterval = endFourthInterval + 1;
    % Sixth and seventh interval: landing acceleration and deceleration
    startSixthInterval = endFifthInterval + 1;
    endSeventhInterval = startSixthInterval + fourthLoopSamples - 1;
    endSixthInterval = fix((startSixthInterval + endSeventhInterval) / 2);
    startSeventhInterval = endSixthInterval + 1;

%     disp(startFirstInterval)
%     disp(endFirstInterval)
%     disp(startSecondInterval)
%     disp(endSecondInterval)
%     disp(startThirdInterval)
%     disp(endThirdInterval)
%     disp(startFourthInterval)
%     disp(endFourthInterval)
%     disp(startFifthInterval)
%     disp(endFifthInterval)
%     disp(startSixthInterval)
%     disp(endSixthInterval)
%     disp(startSeventhInterval)
%     disp(endSeventhInterval)


    % We assume that the reference system for the drone in equal to the NWU
    % (north-west-up) at the initial time t = 0.
    
    % Body linear acceleration
    accBody = zeros(totalNumSamples,3);
    accBody(startFirstInterval:endFirstInterval,3) = 0.2;     % vertical take off: acceleration
    accBody(startSecondInterval:endSecondInterval,3) = -0.2;  % deceleration to have velocity = 0 at the end of the take off
    accBody(startFourthInterval:endFourthInterval,1) = 0.3;     % forward movement: acceleration
    accBody(startFifthInterval:endFifthInterval,1) = -0.3;  % forward movement: deceleration
    accBody(startSixthInterval:endSixthInterval,3) = -0.2;  % vertical landing: acceleration
    accBody(startSeventhInterval:endSeventhInterval,3) = 0.2; % vertical landing: deceleration
    
    % Velocità angolare body
    angVelBody = zeros(totalNumSamples,3);
    angVelBody(startSecondInterval:endSecondInterval,3) = (pi/2)/3; % turn left

elseif selectTrajectory == 2
    disp('Second trajectory selected')
    % Second trajectory: 
    % Vertical take off
    firstLoopSamples = fs*5;    % 500 measurements for the first 5 seconds
    % Turn left (yaw angle)
    secondLoopSamples = fs*3;   % 300 measurements for the next 3 seconds
    % Forward tilt (roll angle) + forward movement
    thirdLoopSamples = fs*30;    % 3000 measurements for the next 30 seconds
    % Backward tilt (roll angle) + forward movement
    fourthLoopSamples = fs*30;    % 3000 measurements for the next 30 seconds
    % Rightward tilt (pitch angle) + rotation around z-axis (yaw angle) + forward motion
    fifthLoopSamples = fs*30;   % 3000 measurements for the next 30 seconds
    % Turn rigth (yaw angle)
    sixthLoopSamples = fs*3;  % 300 measurements for the first 3 seconds
    % Landing
    seventhLoopSamples = fs*5;  % 500 measurements for the first 5 seconds
    totalNumSamples = firstLoopSamples + secondLoopSamples + thirdLoopSamples + fourthLoopSamples + ...
                        fifthLoopSamples + sixthLoopSamples + seventhLoopSamples;
    
    % First and second intervals: take off acceleration and deceleration
    startFirstInterval = 1;
    endFirstInterval = fix((startFirstInterval + firstLoopSamples) / 2);
    startSecondInterval = endFirstInterval + 1;
    endSecondInterval = startSecondInterval + fix((startFirstInterval + firstLoopSamples) / 2) - 1;
    % Third interval: turn left (yaw angle)
    startThirdInterval = endSecondInterval + 1;
    endThirdInterval = startThirdInterval + secondLoopSamples - 1;
    % Fourth and fifth intervals: forward tilt (pitch angle)  + forward movement acceleration
    startFourthInterval = endThirdInterval + 1;
    endFifthInterval = startFourthInterval + thirdLoopSamples - 1;
    endFourthInterval = fix((startFourthInterval + endFifthInterval) / 2);
    startFifthInterval = endFourthInterval + 1;
    % Sixth and seventh intervals: backward tilt (pitch angle) + forward movement deceleration
    startSixthInterval = endFifthInterval + 1;
    endSeventhInterval = startSixthInterval + fourthLoopSamples - 1;
    endSixthInterval = fix((startSixthInterval + endSeventhInterval) / 2);
    startSeventhInterval = endSixthInterval + 1;
    % Eighth and ninth intervals: rightward tilt (roll angle) + rotation around z-axis (yaw angle) + forward motion
    startEighthInterval = endSeventhInterval + 1;
    endNinthInterval = startEighthInterval + fifthLoopSamples - 1;
    endEighthInterval = fix((startEighthInterval + endNinthInterval) / 2);
    startNinthInterval = endEighthInterval + 1;
    % Tenth interval: turn rigth (yaw angle)
    startTenthInterval = endNinthInterval + 1;
    endTenthInterval = startTenthInterval + sixthLoopSamples - 1;
    % Eleventh and twelfth intervals: landing
    startEleventhInterval = endTenthInterval + 1;
    endTwelfthInterval = startEleventhInterval + seventhLoopSamples - 1;
    endEleventhInterval = fix((startEleventhInterval + endTwelfthInterval) / 2);
    startTwelfthInterval = endEleventhInterval + 1;

%     disp(startFirstInterval)
%     disp(endFirstInterval)
%     disp(startSecondInterval)
%     disp(endSecondInterval)
%     disp(startThirdInterval)
%     disp(endThirdInterval)
%     disp(startFourthInterval)
%     disp(endFourthInterval)
%     disp(startFifthInterval)
%     disp(endFifthInterval)
%     disp(startSixthInterval)
%     disp(endSixthInterval)
%     disp(startSeventhInterval)
%     disp(endSeventhInterval)
%     disp(startEighthInterval)
%     disp(endEighthInterval)
%     disp(startNinthInterval)
%     disp(endNinthInterval)
%     disp(startTenthInterval)
%     disp(endTenthInterval)
%     disp(startEleventhInterval)
%     disp(endEleventhInterval)
%     disp(startTwelfthInterval)
%     disp(endTwelfthInterval)

    
    % Body linear acceleration
    accBody = zeros(totalNumSamples,3);
    accBody(startFirstInterval:endFirstInterval,3) = 0.2;   % take off acceleration
    accBody(startSecondInterval:endSecondInterval,3) = -0.2;   % take off deceleration
    accBody(startFourthInterval:endFourthInterval,1) = 0.10; % forward acceleration
    accBody(startSeventhInterval:endSeventhInterval,1) = -0.10; % forward deceleration
    accBody(startEighthInterval:endEighthInterval,1) = 0.10; % forward acceleration
    accBody(startNinthInterval:endNinthInterval,1) = -0.10;  % forward deceleration
    accBody(startEleventhInterval:endEleventhInterval,3) = -0.2;    % landing acceleration
    accBody(startTwelfthInterval:endTwelfthInterval,3) = 0.2;   % landing deceleration

    
    % Velocità angolare body
    angVelBody = zeros(totalNumSamples,3);
    angVelBody(startThirdInterval:endThirdInterval,3) = (pi/2)/3;   % turn left
    angVelBody(startFourthInterval:endFourthInterval,2) = -(pi/4) / 15; % -5° inclination around y-axis in 15 seconds
    angVelBody(startFifthInterval:endFifthInterval,2) = (pi/4) / 15; % 5° inclination around y-axis in 15 seconds
    angVelBody(startSixthInterval:endSixthInterval,2) = (pi/4) / 15; % 5° inclination around y-axis in 15 seconds
    angVelBody(startSeventhInterval:endSeventhInterval,2) = -(pi/4) / 15; % -5° inclination around y-axis in 15 seconds
    angVelBody(startEighthInterval:endEighthInterval,1) = (pi/4) / 15;  % 45° inclination around x-axis in 15 seconds  
    angVelBody(startNinthInterval:endNinthInterval,1) = -(pi/4) / 15;  % -45° inclination around x-axis in 15 seconds
    angVelBody(startEighthInterval:endNinthInterval,3) = -(pi/2) / 30; % turn right
    angVelBody(startTenthInterval:endTenthInterval,3) = -(pi/2) / 3;  % turn right
end

% Definition of kinematicTrajectory object
initPosition = [0,0,0]; % at time t=0 body RS and navigation RS overlap
initVel = [0,0,0];  % at time t=0 the drone is stationary

% Body reference system is rotated by 180° around x and 180° with respect to 
% the navigation reference system (parametrization with roll-pitch-yaw ZYX)

% Orientation matrix (composition in local axes):
yaw = 0;
pitch = 0;
roll = pi;
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

[position,orientationNED,velocity,accNED,angVelNED] = traj(accBody,angVelBody);

% Plot the trajectory in navigation frame (NED frame)
t = (0:(totalNumSamples-1))/fs;

figure(1)
plot3(position(:,1),position(:,2),position(:,3))
xlabel('North (m)')
ylabel('East (m)')
zlabel('Down (m)')
title('Position')
grid on

figure(2)
plot(t,position(:,3))

%% Generate IMU data without noise
% Create an imuSensor object with ideal accelerometer, magnetometer and gyroscope.
IMU = imuSensor('accel-gyro-mag','SampleRate',fs);
[accelReading,gyroReading,magReading] = IMU(-accNED,angVelNED,orientationNED);

% Plot the accelerometer readings, gyroscope readings, and magnetometer readings.
t = (0:(totalNumSamples-1))/IMU.SampleRate;

figure(2)
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
constantBias = [0.01, 0.01, 0.005]; % gyro constant bias [rad]
IMU_Noise.Gyroscope.NoiseDensity = noiseDensity;   % add white noise drift for gyro measurements
IMU_Noise.Gyroscope.ConstantBias = constantBias;    % add a constant bias for gyro measurements (https://www.ericcointernational.com/inertial-measurement-units)

[accelReadingN,gyroReadingN,magReadingN] = IMU_Noise(-accNED,angVelNED,orientationNED);

% Plot the accelerometer readings, gyroscope readings, and magnetometer readings.
t = (0:(totalNumSamples-1))/IMU_Noise.SampleRate;

figure(3)
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