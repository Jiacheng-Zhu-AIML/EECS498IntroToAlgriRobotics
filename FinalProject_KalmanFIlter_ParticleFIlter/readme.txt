Umich EECS498 Introduction to Algorithmic Robotics Fall 2017
Instructor: Dmitry Berenson
GSI: Kevin French

Openrave environment for demonstration

Xiaoke Wang, Jiacheng Zhu


KalmanFilter_check_SimulationCloseLoop.py
Demonstration of Kalman Filter with Gaussian Sensor and Collision checker

KalmanFilter_SimulationCloseLoop.py
Demonstration of Kalman Filter with Gaussian Sensor

ParticleFilter_check_SimulationCloseLoop.py
Demonstration of Particle Filter with Gaussian Sensor and Collision checker

ParticleFilter_SimulationCloseLoop.py
Demonstration of Particle Filter with Gaussian Sensor

In the above simulation, the robot is close-loop controlled.
The robot is given a target, and uses the filtered result as it's own position and then
walks towards the target until reach.

ParticleFilter_Sonar_SimulationOpenLoop.py
Kidnapped Problem
Particle Filter with Sonar Sensor and Collision checker
The robot is open loop controlled, It has no idea of its initial location, and a set of
motions is given to the robot.