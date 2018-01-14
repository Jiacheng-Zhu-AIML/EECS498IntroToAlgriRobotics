

from numpy import *
import time
import numpy as np
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    import numpy
from MoveFunction import *

def GaussianSensor(env, robot, alpha = 1):

    pos = [robot.GetTransform()[0][3], robot.GetTransform()[1][3]]
    pos_new = np.array([0., 0.])
    pos_new[0] = pos[0] + alpha * np.random.randn()
    pos_new[1] = pos[1] + alpha * np.random.randn()
    sigma = 1 / alpha

    return pos_new, sigma

def SonarSensorRobot(env, robot, step = 0.1, maxx = 300):

    print "Sonar robot works"

    local_pos = [robot.GetTransform()[0][3], robot.GetTransform()[1][3]]
    sonar_result = [0, 0, 0, 0]

    for i in range(4):
        if i == 0:
            detect_vector = array([1, 0])
        elif i == 1:
            detect_vector = array([0, -1])
        elif i == 2:
            detect_vector = array([-1, 0])
        elif i == 3:
            detect_vector = array([0, 1])

        for j in range(maxx):

            detect_pos = [local_pos[0] + j*detect_vector[0]*step, local_pos[1] + j*detect_vector[1]*step]

            if CheckCollision(env, robot,detect_pos[0], detect_pos[1], 0):

                sonar_result[i] = (j - 1)*step
                break
            sonar_result[i] = maxx*step

    return sonar_result


def SonarSensorRobotPosition(env, robot, x, y, step=0.1, maxx=300):

    print "Sonor works for [", x, ", ", y, " ]"

    local_pos = [x, y]
    sonar_result = [0, 0, 0, 0]

    for i in range(4):
        if i == 0:
            detect_vector = array([1, 0])
        elif i == 1:
            detect_vector = array([0, -1])
        elif i == 2:
            detect_vector = array([-1, 0])
        elif i == 3:
            detect_vector = array([0, 1])

        for j in range(maxx):

            detect_pos = [local_pos[0] + j*detect_vector[0]*step, local_pos[1] + j*detect_vector[1]*step]

            if CheckCollision(env, robot, detect_pos[0], detect_pos[1], 0):
                sonar_result[i] = (j - 1) * step
                break
            sonar_result[i] = maxx * step

    return sonar_result

