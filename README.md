# Telenav Autonomous Car Developer - Intern Coding Challenge
----

## Problem Statement
---

Sensor Fusion Project:
 - This project consists of implementation of Sensor Fusion Algorithm using filter with C++. 
 - You can use any of EKF, UKF, Kalman Filter, Particle Filter for your implementation.
 - You have been provided with an input file(SensorFusion-data-1.txt) which contains measurements from real sensors like Radar and Lidar and your task is to output fusion of those input measurements.


## Project Description:
---
 - Input file contains iterations. At each iteration, multiple sensor measurments have been taken. 

 - You have to fuse only measurements from different sensors. Sensor Id :  2 = Radar, Sensor Id : 4 = Lidar

 - You can fuse measurements based upon Euclidean Distance between measurements. (Criteria: Euclidean Distance <= 10.0 & absolute delta y = 1.0)

 - From each iteration you should only pick up two different sensor measurements for fusion. If there are no measurements available to fuse, you ca- n skip the iteration.

 - You need to implement filter in C++, so that you can apply these measurements to produce fused measurements.

 - Lidar Measurement Covariance is 0.02 meter.
 - Radar Measurement Covariance is 0.1 meter.

 - You can assume linearity between measurements during individual iteration.



