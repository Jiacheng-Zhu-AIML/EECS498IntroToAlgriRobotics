#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
EECS498 HW03 A-star
JiachengZhu jiachzhu@umich,edu

'A-Star Path Finding Algorithm '

Need to be modified
'''
import time
import openravepy

#### YOUR IMPORTS GO HERE ####

from Astar_Path_finding import *
import numpy as np
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)


def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [x_i, y_i, theta_i]

    if not path:
	return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')	
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
	traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);


    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        goalconfig = [2.6,-1.3,-pi/2]
	start = time.clock()
        #### YOUR CODE HERE ####


    robot_position = robot.GetTransform()
    startconfig = [robot_position[0][3], robot_position[1][3], 0]

    result = AstarPathFinding(0.1,0.1,np.pi/4,startconfig,goalconfig,env)
    print 'result = ', result


    #     #### Implement your algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.

    #     #### Draw the X and Y components of the configurations explored by your algorithm
    path = result[0]  #put your final path in this variable
    print 'The path list is ',path
    print 'len(path) = ',len(path)

    draw_one = PaintPoint(path,env,0,0,0,7)
    detected = result[1]
    print 'The detected list is',detected

    print 'the final cost of this path' ,result[2]
    draw_two = PaintPoint(detected,env,0,0,1,5)
    draw_three = PaintPoint(result[3],env,1,0,0,5)
    draw_one = PaintPoint(path, env, 0, 0, 0, 7)
    #print 'path = ',path
    #### END OF YOUR CODE ###
    end = time.clock()
    print "Time: ", end - start

        # Now that you have computed a path, convert it to an openrave trajectory 
    traj = ConvertPathToTrajectory(robot, path)

	# Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)


    waitrobot(robot)

    raw_input("Press enter to exit...")

