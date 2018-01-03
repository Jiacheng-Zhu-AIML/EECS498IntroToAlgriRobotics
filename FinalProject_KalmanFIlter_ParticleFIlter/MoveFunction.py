'''
Umich EECS498 Introduction to Algorithmic Robotics Fall 2017
Instructor: Dmitry Berenson
GSI: Kevin French

Openrave environment for demonstration

Move Function

Xiaoke Wang, Jiacheng Zhu
'''
from numpy import *
import time
import numpy as np
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    import numpy

from PlotingTools import *



class RobotMotion:
    def __init__(self, env, rob):

        self.environment = env
        self.robot = rob
        self.robot.SetActiveDOFs([],DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, [0, 0, 1])
        self.working = False
        self.moving = False
        self.heading = [0, 0]
        self.open_loop_plan_list = []
        self.close_loop_target_list = []
        self.detatime = 0.1

    def MoveAsPlaned(self):
        '''
        open_loop_plan = [[[1, 0], 10, 0.2], [[0, -1], 10, 0.2], [[1, 0],10,0.2], [[1,1], 40, 0.2]]
        :return:
        '''
        if not self.open_loop_plan_list:
            self.working = False
            return False
        elif self.open_loop_plan_list[0][1] == 0:
            del self.open_loop_plan_list[0]
            self.MoveAsPlaned()
        else:
            temp_plan = self.open_loop_plan_list[0]
            self.MoveAlongVector(temp_plan[0], 1, temp_plan[2],temp_plan[2])
            self.open_loop_plan_list[0][1] = self.open_loop_plan_list[0][1] - 1


    def MoveTorwardTarget(self, now_position, stepsize,threshold,sigma):

        if not self.close_loop_target_list:
            self.working = False
            print 'bye\n'
            return False
        elif self.ReachedTarget(now_position, self.close_loop_target_list[0], stepsize):
            del self.close_loop_target_list[0]
            print 'again\n'
            self.MoveTorwardTarget(now_position, stepsize, threshold,sigma)
        else:
            temp_target = self.close_loop_target_list[0]
            temp_head = [temp_target[0]- now_position[0], temp_target[1] - now_position[1]]
            self.MoveAlongVector(temp_head,1,stepsize,sigma)
            print 'work\n'
        print self.TrueLocation()

    def TrueLocation(self):

        robot = self.robot
        true_location = [robot.GetTransform()[0][3], robot.GetTransform()[1][3]]
        return true_location


    def MoveAlongVector(self, vector, steps, steplength, sigma):
        '''
        Basic move operator
        :param vector:
        :param steps:
        :param steplength:
        :return:
        '''
        self.working = True
        self.moving = True
        self.heading = vector
        detatime = self.detatime
        env = self.environment
        robot = self.robot
        while not robot.GetController().IsDone():
            time.sleep(0.01)
        # print 'before',self.heading
        deta_vector_list = [steplength * vector[0] / linalg.norm(array(vector)),
                            steplength * vector[1] / linalg.norm(array(vector))]
        self.heading = [vector[0] / linalg.norm(array(vector)),
                        vector[1] / linalg.norm(array(vector))]
        # print 'after', self.heading
        last_position = [robot.GetTransform()[0][3], robot.GetTransform()[1][3], robot.GetTransform()[2][3]]
        path = [last_position]
        for i in range(steps):
            this_position = [last_position[0] + deta_vector_list[0] + sigma*np.random.randn(), last_position[1] + deta_vector_list[1] + sigma*np.random.randn(), 0]
            path.append(this_position)
            print 'check',CheckCollision(env, robot, this_position[0], this_position[1], this_position[2])
            if CheckCollision(env, robot, this_position[0], this_position[1], this_position[2]):
                self.moving = False
                time.sleep(detatime)
                return False
            # print 'self.working',self.working
        # this_position = [last_position[0] + deta_vector_list[0] + sigma*np.random.randn(), last_position[1] + deta_vector_list[1] + sigma*np.random.randn(), 0]
        # if CheckCollision(env, robot, this_position[0], this_position[1], this_position[2]):
        #     self.moving = False
        # path.append(this_position)

        traj = ConvertPathToTrajectory(env, robot, path)
        # Execute the trajectory on the robot.
        if traj != None:
            robot.GetController().SetPath(traj)
        time.sleep(detatime)



    def ReachedTarget(self, now_position, goal_list, error = 0.1):

        distance = ((now_position[0] - goal_list[0])**2 + (now_position[1] - goal_list[1])**2)**0.5
        if distance <= error:
            #self.moving = False
            return True
        return False


    # def MoveAlongVectorList(self, vector_list, steps_list, steplength = 0.2):
    #     '''
    #     Seems useless, it can not output heading
    #     :param vector_list:
    #     :param steps_list:
    #     :param steplength:
    #     :return:
    #     '''
    #
    #     path = []
    #     env = self.environment
    #     robot = self.robot
    #     while not robot.GetController().IsDone():
    #         time.sleep(0.01)
    #     self.working = True
    #
    #     last_position = [robot.GetTransform()[0][3], robot.GetTransform()[1][3]]
    #
    #     for i, one in enumerate(vector_list):
    #         for j in range(steps_list[i]):
    #             deta_vector_list = [steplength * vector_list[i][0] / linalg.norm(array(vector_list[i])),
    #                                 steplength * vector_list[i][1] / linalg.norm(array(vector_list[i]))]
    #             this_position = [last_position[0] + deta_vector_list[0], last_position[1] + deta_vector_list[1], 0]
    #             path.append(this_position)
    #             last_position = this_position
    #
    #     traj = ConvertPathToTrajectory(env, robot, path)
    #     # Execute the trajectory on the robot.
    #     if traj != None:
    #         robot.GetController().SetPath(traj)
    #
    #     self.working = False


def ConvertPathToTrajectory(env, robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [x_i, y_i, theta_i]

    if not path:
	return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        # print path[i]
        # path[i] = path[i]+[0.]
        # print path[i]
        traj_temp = traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=10*ones(3),maxaccelerations=10*ones(3))
    return traj

# def waitrobot(robot):
#     """busy wait for robot completion"""
#     while not robot.GetController().IsDone():
#         time.sleep(0.01)

# def check(nextstep,robot,env):
#     pos_affine = robot.GetTransform()
#     dirct = dirc(np.array([pos_affine[0][3],pos_affine[1][3]]),nextstep)
#     result = []
#     while (pos_affine[0][3]-nextstep[0])**2+(pos_affine[1][3]-nextstep[1])**2>0.005:
#         pos_affine[0][3] = pos_affine[0][3]+dirct[0]*0.05
#         pos_affine[1][3] = pos_affine[1][3]+dirct[1]*0.05
#         with env:
#             robot.SetTransform(pos_affine)
#             result.append(env.CheckCollision(robot))
#     return any(result)

def CheckCollision(env, robot, x, y, theta=0):

    #print 'Using DetectPointCollision'
    position = [x, y, theta]
    target_robot = position
    robot_position = robot.GetTransform()

    Tz = matrixFromAxisAngle([0, 0, position[2]])

    Tz[0][3] = target_robot[0]
    Tz[1][3] = target_robot[1]
    Tz[2][3] = robot_position[2][3]
        # Tz[2][3] = target_robot[2] - robot_position[2][3]
    with env:
        robot.SetTransform(Tz)
        result = env.CheckCollision(robot)
        robot.SetTransform(robot_position)
    #print 'in collision func, Now robot.GetTransform() = ', robot.GetTransform()
    return result
