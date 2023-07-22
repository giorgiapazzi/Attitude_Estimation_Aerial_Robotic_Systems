# Attitude_Estimation_Aerial_Robotic_Systems
Implementation of "Explicit Complementary Filter" for attitude estimation of aerial robotic systems based on IMU measurements.
The project has been realized for the course of Guidance and Navigation Systems at University of Pisa (Course Of Studies in Robotics).

# Requirements
MATLAB 2022b or later versions.

# Instructions
To create the simulated IMU measurements run the "IMU_data_generation.m" script. Inside the script can be setted different trajectories (which simulate the flight of a VTOL drone) and different initial body frame orientations with respect to navigation frame.
To compute the attitude estimation from rotation matrix and from Euler's angle kinematics run the "mechanization.m" script.
To compute attitude estimation with Explicit Complementary Filter run the "explicit_complementary_filter.m" script.

## References
<a id="1">[1]</a> 
Minh-Duc Hua, Guillaume Ducard, Tarek Hamel, Robert Mahony. Introduction to Nonlinear Attitude
Estimation for Aerial Robotic Systems. Aerospace Lab, 2014, pp.AL08-04. ￿10.12762/2014.AL08-04 .
hal-01113138
