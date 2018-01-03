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

'''
"* No demo.py in submission (They have several different files for different demos)
* Kalman Filter with absolute sensor and no collision detection had higher average error than sensor measurements
* Particle Filter with absolute sensor follows the sensor closely suggesting that there action model has much smaller weight than sensor model
* Robot can get stuck on wall in environment and program does not stop until the robot gets to the other side of the room, therefore the program could run forever
* Particle filter with relative sensor ""Sonar"" has large spread of points but forms one probability mass on top of the true location quickly
* Relatively simple environment for everything except for the kidnapped robot problem using sonar
* Everything ran quickly except sonar based particle filter which ran slow but finished well within 30 minutes
* I only ran what was in the readme.txt"
Write up:
"Overall the report is very good. A few issues:
-Some grammar and word use problems make reading difficult (-3 points). E.g. ""stucked"" is not a word, you probably mean ""stuck.""
-Make sure your references don't have junk in them (like ""to appear"" for a paper that was published 10 years ago). Since references weren't strictly required I am not taking points off for this.
"
'''