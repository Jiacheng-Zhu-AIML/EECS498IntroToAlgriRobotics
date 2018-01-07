# EECS498IntroToAlgriRobotics
Umich EECS498 Introduction to Algorithmic Robotics work selections

Umich EECS498 Introduction to Algorithmic Robotics Fall 2017
Instructor: Dmitry Berenson
GSI: Kevin French


Final Project Problem:
"Localization"
Consider the PR2 robot navigating in an openrave environment with obstacles. Implement a function that 
simulates a simple location sensor in openrave (i.e. the function shouldreturn a slightly noisy estimate 
of the true location). Pick an interesting path for the robot to execute and estimate the robot’s 
position as it executes the path using a) A Kalman filter, and b) A particlefilter. 
You will need to tune the noise in the sensing and action and the parameters of the algorithms to make 
sure there is enough noise to make the problem interesting but not too much so that it’s impossible to 
estimate the location. Compare the performance of the two algorithms in terms of accuracy in several 
interesting scenarios. Produce a case where the Kalman filter is unable to produce a reasonable 
estimate (e.g. the mean is inside an obstacle) but the particle filter does producea reasonable estimate.


Ubuntu and Openrave environment are required for demonstration
http://openrave.org/docs/latest_stable/tutorials/openravepy_beginning/#openravepy-beginning
https://docs.google.com/document/u/1/d/e/2PACX-1vQtAhPZmLgb11cZASTah8CnRaCZydqnul3879rYE7ltcJ7SqF-UGhkxlinzjNUjJ2jK5B3_pUAlYhIF/pub

Xiaoke Wang, Jiacheng Zhu