# 2602H
HydraCode

Code Summary:

Part one:

Auton Algorithms: Gyro Correction, Coordinate Gird Odometry, Slew control (1D Motion Profiling), PID

Updates made to auton functions for this competition: All previous code was integrated with the odometry functions, added gyro correction algorithm. Code was compressed to about 300 lines from 3000, several main functions were combined into one: goTo(float X, float Y);.

Driver Code Algorithms: PID / Velocity control with okapi api, controls lift height for cubes, towers, stacks. PID allows for us to make cube collection macros, assists driver in driver control.

Initialize Algorithms: Gyro Noise Filteration. Init period spends 3000ms collecting data points from the gyroscope, and then applies a filter to reduce any drift. Motor brakes are also initialized in this period.
