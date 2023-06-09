clear all
close all
clc

%% Trajectory generation
% The drone makes a vertical take off and moves with small accelerations

selectTraj = 2;
selectBodyRS = 2;   % 1 is for ENU, 2 is for NWU, 3 is for 45° rotation
fprintf('Selected trajectory: %d \n', selectTraj)
fprintf('Selected body reference system: %d \n', selectBodyRS)

fs = 100;   % Hz samples per second

if selectTraj == 1 
    % First trajectory: 73 seconds of flight
    % take off + turn left + move forward + landing
    firstLoopSamples = fs*5;    % take off 5 seconds
    secondLoopSamples = fs*3;   % turn left 3 seconds
    thirdLoopSamples = fs*60;   % move forward 60 seconds
    fourthLoopSamples = fs*5;   % landing 5 seconds
    totalNumSamples = firstLoopSamples + secondLoopSamples + thirdLoopSamples + fourthLoopSamples;
    
    % take off
    startFirstInterval = 1;
    endFirstInterval = firstLoopSamples/2;
    startSecondInterval = endFirstInterval+1;
    endSecondInterval = startSecondInterval + firstLoopSamples/2 - 1;
    % turn left
    startThirdInterval = endSecondInterval + 1;
    endThirdInterval = startThirdInterval + secondLoopSamples - 1;
    % move forward
    startFourthInterval = endThirdInterval + 1;
    endFifthInterval = startFourthInterval + thirdLoopSamples - 1;
    endFourthInterval = fix((startFourthInterval + endFifthInterval)/2);
    startFifthInterval = endFourthInterval + 1;
    % landing
    startSixthInterval = endFifthInterval + 1;
    endSeventhInterval = startSixthInterval + fourthLoopSamples - 1;
    endSixthInterval = fix((startSixthInterval + endSeventhInterval)/2);
    startSeventhInterval = endSixthInterval + 1;
        
    % Body linear acceleration
    accBody = zeros(totalNumSamples,3);
    accBody(startFirstInterval:endFirstInterval,3) = 0.5;    % vertical take off
    accBody(startSecondInterval:endSecondInterval,3) = -0.5;
    accBody(startFourthInterval:endFourthInterval,1) = 0.005;  % forward movement
    accBody(startFifthInterval:endFifthInterval,1) = -0.005;
    accBody(startSixthInterval:endSixthInterval,3) = -0.5;  % vertical landing
    accBody(startSeventhInterval:endSeventhInterval,3) = 0.5;
    
    % Body angular velocity
    angVelBody = zeros(totalNumSamples,3);
    angVelBody(startThirdInterval:endThirdInterval,3) = (pi/2)/3;    % turn left

elseif selectTraj == 2
    % Second trajectory: 80 seconds of flight
    % take off + move forward with pitch angle oscillation + landing
    takeoffSamples = fs*5;  % take off 5 seconds
    landingSamples = fs*5;  % landing 5 seconds      
    totalNumSamples = 8000;

    % take off
    startFirstInterval = 1;
    endFirstInterval = takeoffSamples/2;
    startSecondInterval = endFirstInterval + 1;
    endSecondInterval = startSecondInterval + takeoffSamples/2 - 1;
    % landing
    startThirdInterval = totalNumSamples - landingSamples + 1;
    endFourthInterval = totalNumSamples;
    endThirdInterval = fix((startThirdInterval + endFourthInterval)/2);
    startFourthInterval = endThirdInterval + 1;

    % Body linear acceleration
    accBody = zeros(totalNumSamples,3);
    accBody(startFirstInterval:endFirstInterval,3) = 0.7;   % vertical take off
    accBody(startSecondInterval:endSecondInterval,3) = -0.7;
    accBody(startThirdInterval:endThirdInterval,3) = -0.7;  % vertical landing
    accBody(startFourthInterval:endFourthInterval,3) = 0.7;
    accBody(endSecondInterval+1:endSecondInterval+200,1) = 0.5;    % forward movement
    accBody(startThirdInterval-200:startThirdInterval-1,1) = -0.5;

    % Sine wave signal for rotation around Y-axis:
    duration = startThirdInterval - endSecondInterval - 1;  % samples in one period
    period = duration/fs;   % s
    dt = 1/fs;  % second per sample
    t = (0:dt:(period-dt));
    Fc = 2*pi/period;   % rad/s
    pitch_rotation = -((pi/4)/(70/4))*sin(Fc*t);
    % Plot the signal versus time:
%     figure(10);
%     plot(t,pitch_rotation);
%     xlabel('time [s]');
%     ylabel('angular velocity Y-axis [rad/s]');
%     title('Angular velocity signal');
%     grid on

    % Body angular velocity
    angVelBody = zeros(totalNumSamples,3);
    angVelBody(endSecondInterval+1:startThirdInterval-1,2) = pitch_rotation';   % pitch angle oscillations

elseif selectTraj == 3
    % Third trajectory: 87 seconds of flight
    % take off + turn left + turn right + move forward + landing
    firstLoopSamples = fs*5;    % take off 5 seconds
    secondLoopSamples = fs*10;  % turn left 10 seconds
    thirdLoopSamples = fs*10;   % turn right in 10 seconds
    fourthLoopSamples = fs*57;  % move forward in 57 seconds
    fifthLoopSamples = fs*5;    % landing in 5 seconds
    totalNumSamples = firstLoopSamples + secondLoopSamples + thirdLoopSamples + fourthLoopSamples + fifthLoopSamples;
    
    % Take off
    startFirstInterval = 1;
    endFirstInterval = firstLoopSamples/2;
    startSecondInterval = endFirstInterval+1;
    endSecondInterval = startSecondInterval + firstLoopSamples/2 - 1;
    % Turn left
    startThirdInterval = endSecondInterval + 1;
    endThirdInterval = startThirdInterval + secondLoopSamples - 1;
    % Turn right
    startFourthInterval = endThirdInterval + 1;
    endFourthInterval = startFourthInterval + thirdLoopSamples - 1;
    % Move forward
    startFifthInterval = endFourthInterval + 1;
    endSixthInterval = startFifthInterval + fourthLoopSamples - 1;
    endFifthInterval = fix((startFifthInterval + endSixthInterval)/2);
    startSixthInterval = endFifthInterval + 1;
    % Landing
    startSeventhInterval = endSixthInterval + 1;
    endEighthInterval = startSeventhInterval + fifthLoopSamples - 1;
    endSeventhInterval = fix((startSeventhInterval + endEighthInterval)/2);
    startEighthInterval = endSeventhInterval + 1;
      
    % Body linear acceleration
    accBody = zeros(totalNumSamples,3);
    accBody(startFirstInterval:endFirstInterval,3) = 0.5;    % vertical take off
    accBody(startSecondInterval:endSecondInterval,3) = -0.5;
    accBody(startFifthInterval:endFifthInterval,1) = 0.005;  % forward movement
    accBody(startSixthInterval:endSixthInterval,1) = -0.005;
    accBody(startSeventhInterval:endSeventhInterval,3) = -0.5;  % vertical landing
    accBody(startEighthInterval:endEighthInterval,3) = 0.5;
    
    % Velocità angolare body
    angVelBody = zeros(totalNumSamples,3);
    angVelBody(startThirdInterval:endThirdInterval,3) = (pi/2)/10;    % turn left
    angVelBody(startFourthInterval:endFourthInterval,3) = -(pi/2)/10;  % turn right
    
elseif selectTraj == 4
    % Sixth trajectory: 73 seconds of flight
    % take off + turn around + move forward + landing
    firstLoopSamples = fs*5;    % take off 5 seconds
    secondLoopSamples = fs*3;   % turn right 3 seconds
    thirdLoopSamples = fs*60;   % move forward 60 seconds
    fourthLoopSamples = fs*5;   % landing 5 seconds
    totalNumSamples = firstLoopSamples + secondLoopSamples + thirdLoopSamples + fourthLoopSamples;
    
    % take off
    startFirstInterval = 1;
    endFirstInterval = firstLoopSamples/2;
    startSecondInterval = endFirstInterval+1;
    endSecondInterval = startSecondInterval + firstLoopSamples/2 - 1;
    % turn around
    startThirdInterval = endSecondInterval + 1;
    endThirdInterval = startThirdInterval + secondLoopSamples - 1;
    % move forward
    startFourthInterval = endThirdInterval + 1;
    endFifthInterval = startFourthInterval + thirdLoopSamples - 1;
    endFourthInterval = fix((startFourthInterval + endFifthInterval)/2);
    startFifthInterval = endFourthInterval + 1;
    % landing
    startSixthInterval = endFifthInterval + 1;
    endSeventhInterval = startSixthInterval + fourthLoopSamples - 1;
    endSixthInterval = fix((startSixthInterval + endSeventhInterval)/2);
    startSeventhInterval = endSixthInterval + 1;
    
    % Body linear acceleration
    accBody = zeros(totalNumSamples,3);
    accBody(startFirstInterval:endFirstInterval,3) = 0.5;    % vertical take off
    accBody(startSecondInterval:endSecondInterval,3) = -0.5;
    accBody(startFourthInterval:endFourthInterval,1) = 0.005;  % forward movement
    accBody(startFifthInterval:endFifthInterval,1) = -0.005;
    accBody(startSixthInterval:endSixthInterval,3) = -0.5;  % vertical landing
    accBody(startSeventhInterval:endSeventhInterval,3) = 0.5;
    
    % Body angular velocity
    angVelBody = zeros(totalNumSamples,3);
    angVelBody(startThirdInterval:endThirdInterval,3) = (2*pi)/3;    % turn around
end


%% Definition of kinematicTrajectory object
initPosition = [0,0,0]; % at time t=0 body frame and navigation frame overlap
initVel = [0,0,0];  % at time t=0 the drone is stationary

if selectBodyRS == 1
    % We assume that the body reference system is ENU (east-north-up)
    % Body reference system is rotated by -90° around z-axis and 180° around y-axis with
    % respect to the navigation reference system (parametrization with
    % yaw-pitch-roll ZYX)
    yaw = -pi/2;
    pitch = pi;
    roll = 0;
elseif selectBodyRS == 2
    % We assume that the body reference system is NWU (north-west-up)
    % Body reference system is rotated by 180° around x-axis with
    % respect to the navigation reference system (parametrization with
    % yaw-pitch-roll ZYX)
    yaw = 0;
    pitch = 0;
    roll = pi;
elseif selectBodyRS == 3
    % We assume that the body reference system is rotated by 45° around
    % z-axis and by 180° around x-axis with respect to the navigation
    % reference system (parametrization with yaw-pitch-roll ZYX)
    yaw = pi/4;
    pitch = 0;
    roll = pi;
end

% Compute rotation matrix from navigation reference system to body
% reference system (composition in local axes)
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
initOrientation = R;

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
ylabel('Acceleration [m/s^2]')
grid on

subplot(3,1,2)
plot(t,gyroReading)
legend('X-axis','Y-axis','Z-axis')
title('Gyroscope Readings')
ylabel('Angular Velocity (rad/s)')
grid on

subplot(3,1,3)
plot(t,magReading)
legend('X-axis','Y-axis','Z-axis')
title('Magnetometer Readings')
xlabel('Time (s)')
ylabel('Magnetic Field (uT)')
grid on


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

noiseDensity = 1e-07;   % gyro white noise drift
%noiseDensity = 1e-07;
%constantBias = [0.01, 0.03, 0.05]; % gyro constant bias 
% constantBias = [1e-03, 3e-03, 5e-03];
%constantBias = [1e-05, 3e-05, 5e-05];
constantBias = [1e-06, 3e-06, 5e-06];
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
grid on

subplot(3,1,2)
plot(t,gyroReadingN)
legend('X-axis','Y-axis','Z-axis')
title('Gyroscope Readings With Noise')
ylabel('Angular Velocity (rad/s)')
grid on

subplot(3,1,3)
plot(t,magReadingN)
legend('X-axis','Y-axis','Z-axis')
title('Magnetometer Readings With Noise')
xlabel('Time (s)')
ylabel('Magnetic Field (uT)')
grid on

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
log_vars.trajectory = selectTraj;
log_vars.frame = selectBodyRS;
save('dataset','log_vars');