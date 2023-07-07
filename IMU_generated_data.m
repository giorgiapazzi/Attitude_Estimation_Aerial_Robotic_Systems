clear all
close all
clc

%% Trajectory generation
% The drone makes a vertical take off and moves with small accelerations

selectTraj = 1;
fs = 100;   % Hz samples per second

if selectTraj == 1 
    % First trajectory: 73 seconds of flight
    % The drone moves in vertical for 1 m in 2.5 s and then rotates 90° in 3 s.
    % It then moves forward for 60 seconds
    % and performs a vertical landing with the same acceleration of the take off.
    firstLoopSamples = fs*5;    % 500 measures for the first 5 seconds
    secondLoopSamples = fs*3;   % 300 measures for the next 3 seconds
    thirdLoopSamples = fs*60;   % 6000 measures for the next 60 seconds
    fourthLoopSamples = fs*5;   % 500 measures for the final seconds of the trajectory
    totalNumSamples = firstLoopSamples + secondLoopSamples + thirdLoopSamples + fourthLoopSamples;
    
    startFirstInterval = 1;
    endFirstInterval = firstLoopSamples/2;
    startSecondInterval = endFirstInterval+1;
    endSecondInterval = startSecondInterval + firstLoopSamples/2 - 1;
    startThirdInterval = endSecondInterval + 1;
    endThirdInterval = startThirdInterval + secondLoopSamples - 1;
    startFourthInterval = endThirdInterval + 1;
    endFifthInterval = startFourthInterval + thirdLoopSamples - 1;
    endFourthInterval = fix((startFourthInterval + endFifthInterval)/2);
    startFifthInterval = endFourthInterval + 1;
    startSixthInterval = endFifthInterval + 1;
    endSeventhInterval = startSixthInterval + fourthLoopSamples - 1;
    endSixthInterval = fix((startSixthInterval + endSeventhInterval)/2);
    startSeventhInterval = endSixthInterval + 1;
    
    % We assume that the reference system for the drone in equal to the ENU
    % (east-north-up) at the initial time t = 0.
    
    % Body linear acceleration
    accBody = zeros(totalNumSamples,3);
    accBody(startFirstInterval:endFirstInterval,3) = 0.5;    % vertical take off
    accBody(startSecondInterval:endSecondInterval,3) = -0.5;
    accBody(startFourthInterval:endFourthInterval,1) = 0.005;  % forward movement
    accBody(startFifthInterval:endFifthInterval,1) = -0.005;
    accBody(startSixthInterval:endSixthInterval,3) = -0.5;  % vertical landing
    accBody(startSeventhInterval:endSeventhInterval,3) = 0.5;
    
    % Velocità angolare body
    angVelBody = zeros(totalNumSamples,3);
    angVelBody(startThirdInterval:endThirdInterval,3) = (pi/2)/3;    % turn left

elseif selectTraj == 2
    % Second trajectory
    takeoffSamples = fs*5;    % vertical take off
    landingSamples = fs*5;   % vertical landing
    totalNumSamples = 8000; % 80 seconds of flight

    startFirstInterval = 1;
    endFirstInterval = takeoffSamples/2;
    startSecondInterval = endFirstInterval + 1;
    endSecondInterval = startSecondInterval + takeoffSamples/2 - 1;
    startThirdInterval = totalNumSamples - landingSamples + 1;
    endFourthInterval = totalNumSamples;
    endThirdInterval = fix((startThirdInterval + endFourthInterval)/2);
    startFourthInterval = endThirdInterval + 1;

    accBody = zeros(totalNumSamples,3);
    accBody(startFirstInterval:endFirstInterval,3) = 0.7;
    accBody(startSecondInterval:endSecondInterval,3) = -0.7;
    accBody(startThirdInterval:endThirdInterval,3) = -0.7;
    accBody(startFourthInterval:endFourthInterval,3) = 0.7;
    accBody(endSecondInterval+1:endSecondInterval+200,1) = 0.2;
    accBody(startThirdInterval-200:startThirdInterval-1,1) = -0.2;

    % Sine wave signal for rotation around X-axis:
%     dt = 1/fs;  % second per sample
%     stopTime = 74;
%     t = (0:dt:stopTime-dt);
%     Fc = 0.001;
%     roll = (pi/6)*sin(2*pi*Fc*t);
%     % Plot the signal versus time:
%     figure;
%     plot(t,roll);
%     xlabel('time (in seconds)');
%     title('Signal versus Time');
%     zoom xon;

    % Sine wave signal for rotation around Y-axis:
    duration = startThirdInterval - endSecondInterval - 1;  % samples in one period
    period = duration/fs;   % s
    dt = 1/fs;  % second per sample
    t = (0:dt:(period-dt));
    Fc = 2*pi/period;   % rad/s
    pitch_rotation = -((pi/4)/(70/4))*sin(Fc*t);
%   % Plot the signal versus time:
%     figure(10);
%     plot(t,pitch_rotation);
%     xlabel('time [s]');
%     ylabel('angular velocity Y-axis [rad/s]');
%     title('Angular velocity signal');
%     zoom xon;

    % Sine wave signal for rotation around Z-axis:
%     dt = 1/fs;  % second per sample
%     stopTime = 74;
%     t = (0:dt:stopTime-dt);
%     Fc = 0.005;
%     yaw = (2*pi/70)*sin(2*pi*Fc*t);
%     % Plot the signal versus time:
%     figure;
%     plot(t,yaw);
%     xlabel('time (in seconds)');
%     title('Signal versus Time');
%     zoom xon;

    angVelBody = zeros(totalNumSamples,3);
    angVelBody(endSecondInterval+1:startThirdInterval-1,2) = pitch_rotation';
end


%% Definition of kinematicTrajectory object
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
Rx = [  1   0           0;
        0   cos(roll)   sin(roll);
        0   -sin(roll)  cos(roll)];     % rotation around x axis     
%R = Rz*Ry*Rx;  % composition from left to right: rotation matrix from navigation frame to body frame
R = Rx' * Ry' * Rz';    % composition from right to left: rotation matrix from navigation frame to body frame
initOrientation = R;    % rotation matrix from navigation frame to body frame

traj = kinematicTrajectory('SampleRate',fs,...
    'Velocity',initVel,...
    'Position',initPosition,...
    'Orientation',initOrientation);

[position,orientationNED,~,accNED,angVelNED] = traj(accBody,angVelBody);

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
plot(t,position(:,1))
hold on
plot(t,position(:,2))
hold on
plot(t,position(:,3))
xlabel('Time (s)')
ylabel('Space (m)')
title('Position')
legend('X-axis','Y-axis','Z-axis')
grid on

%% Generate IMU data without noise
% Create an imuSensor object with ideal accelerometer, magnetometer and gyroscope.
IMU = imuSensor('accel-gyro-mag','SampleRate',fs);
[accelReading,gyroReading,magReading] = IMU(-accNED,angVelNED,orientationNED);

% Plot the accelerometer readings, gyroscope readings, and magnetometer readings.
t = (0:(totalNumSamples-1))/IMU.SampleRate;

figure(3)
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

noiseDensity = 1e-05;   % gyro white noise drift
%constantBias = [0.01, 0.01, 0.005]; % gyro constant bias 
constantBias = [1e-03, 1e-03, 5e-04];
IMU_Noise.Gyroscope.NoiseDensity = noiseDensity;   % add white noise drift for gyro measurements
IMU_Noise.Gyroscope.ConstantBias = constantBias;    % add a constant bias for gyro measurements (https://www.ericcointernational.com/inertial-measurement-units)

[accelReadingN,gyroReadingN,magReadingN] = IMU_Noise(-accNED,angVelNED,orientationNED);

% Plot the accelerometer readings, gyroscope readings, and magnetometer readings.
t = (0:(totalNumSamples-1))/IMU_Noise.SampleRate;

figure(4)
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
log_vars.initOrientation = initOrientation';
for i=1 : totalNumSamples
    orientationBODY(:,:,i) = orientationNED(:,:,i)';
end
log_vars.orientation = orientationBODY;
log_vars.frequency = fs;
log_vars.numSamples = totalNumSamples;
log_vars.gyrobias = constantBias;
save('dataset','log_vars');