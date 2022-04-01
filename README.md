# EE4308 Project 1 Lab

## Team Members:
1. Lin Zhicheng [A0199536L]
2. Rishab Patwari [A0184456W]
3. Ong Jia Yuan [A0201643U]
4. Nguyen Tuan Dung [A0201264W]

## Project Tasks:
1. Design a FSM and Generate Trajectories for Pure Pursuit in `ee4308_hector/src/main.cpp`
    - Use look-ahead distance or duration to gen targets from traj
    - Use Cubic Spline Trajectories & Predict the turtle's motion when generating these traj
    - Justify ALL Param
<br>
<br>
2. Design a PID motion controller in `ee4308_hector/src/move.cpp`
    - Satisfy all motion requirements above
    - Justify the GAINS used. Use data collected while using the ground truth for the hector
<br>
<br>
3. Program an extended Kalman Filter (EKF) to estimate the pose of the hector in `ee4308_hector/src/motion.cpp`
    - Justify and collect data for the noise values used
    - 4-man teams must implement
      - GPS
      - Magnetometer
      - Barometer
      - Sonar




