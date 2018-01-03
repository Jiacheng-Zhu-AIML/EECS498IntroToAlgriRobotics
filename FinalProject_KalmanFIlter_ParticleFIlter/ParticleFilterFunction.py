'''
Umich EECS498 Introduction to Algorithmic Robotics Fall 2017
Instructor: Dmitry Berenson
GSI: Kevin French

Openrave environment for demonstration

Particle Filter functions

Xiaoke Wang, Jiacheng Zhu
'''

from numpy import *
import time
import numpy as np
import openravepy
from numpy.random import randn
from numpy.random import uniform
if not __openravepy_build_doc__:
    from openravepy import *
    import numpy

from PlotingTools import *
from MoveFunction import *
from SensorFunction import *

class Particles:
    def __init__(self, environment, robot):
        self.particles = matrix([])
        self.env = environment
        self.robot = robot
        self.N = 0
        self.M = 0
        self.resample_gaussian_sigma_list = [0.5, 0.5]

    def InitializeParticels(self, x_range, y_range, N = 1000, M = 3):

        self.N = N
        self.M = M
        self.particles = np.empty((N, M))

        self.particles[:, 0] = uniform(x_range[0], x_range[1], size=N)
        self.particles[:, 1] = uniform(y_range[0], y_range[1], size=N)
        self.particles[:, -1] = 1 / N

        return self.particles

    def InitializeParticelsCollision(self, x_range, y_range, N=1000, M=3):

        self.N = N
        self.M = M
        self.particles = np.empty((N, M))

        for one in self.particles:

            while 1:
                temp = np.empty((1, M))
                temp[0][0] = uniform(x_range[0], x_range[1])
                temp[0][1] = uniform(y_range[0], y_range[1])

                if not CheckCollision(self.env, self.robot, temp[0][0], temp[0][1]):
                    one[0] = temp[0][0]
                    one[1] = temp[0][1]
                    break
        self.particles[:, -1] = 1 / N

        return self.particles

    def PredictMove(self,head, stepsize = 1):

        # for each particle , apply a probabilistic head
        # need to be modified
        # self.particles[:, 0] += head[0]
        # self.particles[:, 1] += head[1]
        for one in self.particles:

            next_x = one[0]+head[0]*stepsize
            next_y = one[1]+head[1]*stepsize

            if not CheckCollision(self.env, self.robot, next_x, next_y):
                one[0] = next_x
                one[1] = next_y

        return self.particles

    def UpdateWeight(self, senserinput, weightingmethod):

        for one in self.particles:
            weight_temp = weightingmethod(one, senserinput)
            one[-1] = weight_temp

        self.particles = self.particles[np.lexsort(-self.particles.T)]

        return self.particles

    def UpdateWeightSonar(self, senserinput, weightingmethod):

        for one in self.particles:

            weight_temp = weightingmethod(self.env, self.robot, one, senserinput)
            one[-1] = weight_temp

        self.particles = self.particles[np.lexsort(-self.particles.T)]

        return self.particles


    def Resample(self, ResampleMethod,reserve=0.80):

        self.particles = self.particles[np.lexsort(-self.particles.T)]
        number = len(self.particles)

        cut_length = int(reserve * number)
        self.particles = self.particles[: cut_length]
        weight_range = max(self.particles[:, 2]) - min(self.particles[:, 2])

        while len(self.particles) < number:
            for i in range(cut_length):
                threshold = np.random.random() * weight_range
                if self.particles[i][-1] > threshold:

                    new_particle = ResampleMethod(self.env, self.robot, self.particles[i], self.resample_gaussian_sigma_list, 1, self.M)

                    if not CheckCollision(self.env, self.robot, new_particle[0][0], new_particle[0][1], 0):
                        self.particles = np.append(self.particles, new_particle, axis=0)
                break

        self.particles[:, -1] = self.particles[:, -1] / np.sum(self.particles, axis=0)[-1]

        return self.particles

    def Estimate_XY(self):

        mean_pos_array = np.mean(self.particles, axis=0)
        result_pos = np.array([mean_pos_array[0], mean_pos_array[1]])

        return result_pos
    def EstimateByWeight(self):

        x_sum = 0
        y_sum = 0
        w_sum = 0
        for one in self.particles:
            x_sum = x_sum + one[0]*one[-1]
            y_sum = y_sum + one[1]*one[-1]
            w_sum = w_sum + one[-1]
        result_pos = np.array([x_sum/w_sum, y_sum/w_sum])

        return result_pos

def GaussianResample(env, robot, mu_xy_list, sigma_xy_array, N, M):

    particles = np.empty((N, M))
    particles[:, 0] = mu_xy_list[0] + (randn(N) * sigma_xy_array[0])
    particles[:, 1] = mu_xy_list[1] + (randn(N) * sigma_xy_array[1])
    particles[:, -1] = 0

    return particles

def EcliudWeighting(one_particle_list_array, sensor_list_arry):
    '''

    :param one_particle_list_array: [...,[x, y, w],..]
    :param target_list_arry:[x_s, y_s]
    :return: w (new weight according to Euclidean distance
    '''
    new_w = ((one_particle_list_array[0]-sensor_list_arry[0])**2+(one_particle_list_array[1]-sensor_list_arry[1])**2)**0.5

    new_w = 10.0/new_w
    return new_w

def SonarWeighting(env, robot, one_particle, sonar_senser_array):

    one_particle_sonar_list = SonarSensorRobotPosition(env,robot, one_particle[0], one_particle[1])

    new_w = linalg.norm(array(one_particle_sonar_list) - array(sonar_senser_array))

    new_w = 20.0/(new_w + 0.0001)

    return new_w