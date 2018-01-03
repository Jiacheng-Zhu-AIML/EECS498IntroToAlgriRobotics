#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import numpy
#### YOUR IMPORTS GO HERE ####
from numpy import *
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
        jointnames = ['torso_lift_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([0.24,1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

#set active DOF values from a numpy matrix
def SetActiveDOFValuesNPMatrix(robot,qmat):
    qo = [q.item(i) for i in range(0,qmat.shape[1])]
    robot.SetActiveDOFValues(qo)


#returns the end effector transform in the world frame
def GetEETransform(robot,activedofvalues=None): #
	if activedofvalues != None:
	    robot.SetActiveDOFValues(activedofvalues);
	manip = robot.GetActiveManipulator()
	return manip.GetEndEffectorTransform()

#returns the joint axis in the world frame
def GetJointAxis(robot,jointname): #<type 'numpy.ndarray'>
    return robot.GetJoint(jointname).GetAxis(0)

#returns the joint position in the world frame
def GetJointPosition(robot,jointname):#<type 'numpy.ndarray'>
    return robot.GetJoint(jointname).GetAnchor()

def GetTranslationJacobian(robot,jointnames):
    J = numpy.zeros((3,robot.GetActiveDOF()))
    ### YOUR CODE HERE ###

    for ji in range(0,len(jointnames)):
        ee_tf_mat = GetEETransform(robot)
        jt_xyz_ary = GetJointPosition(robot,jointnames[ji])
        ee_xyz_ary = array([ee_tf_mat[0][3], ee_tf_mat[1][3], ee_tf_mat[2][3]])
        p_xyz_ary = array([ee_xyz_ary[0]-jt_xyz_ary[0], ee_xyz_ary[1]-jt_xyz_ary[1], ee_xyz_ary[2]-jt_xyz_ary[2]])
        v_rpy_ary = GetJointAxis(robot, jointnames[ji])
        crs_pdt_ary = cross(v_rpy_ary, p_xyz_ary)

        for jj in range(0,3):
            J[jj][ji] = crs_pdt_ary[jj]

    #print 'New J using VxP is'
    #print J
    #
    #
    # print 'robot.GetActiveDOF() =', robot.GetActiveDOF()
    # print 'robot.GetActiveDOFValues()  ', robot.GetActiveDOFValues()
    #test_result = GetEETransform(robot)
    #print 'GetEETransform(robot) = ', test_result
    robot.GetController().SetDesired(robot.GetDOFValues());
    ### YOUR CODE HERE ###
    return J


def GetJpinv(J):
    ### YOUR CODE HERE ###
    lmbd = 0.01
    J_np_mat = matrix(J)

    i_mat = matrix(eye( J_np_mat.shape[1]))
    J_np_mat_t = J_np_mat.T
    Jpinv = ((J_np_mat_t*J_np_mat + lmbd*lmbd*i_mat)**-1)*J_np_mat_t

    #print 'Jpinv =', Jpinv
    ### YOUR CODE HERE ###
    return Jpinv


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    # env.SetDebugLevel(1) #comment this in to turn off openrave warnings
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from environment XML file
    env.Load('pr2only.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms and raise torso
    tuckarms(env,robot);
  
    #set start config
    robot.SetActiveManipulator('leftarm')
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_upper_arm_roll_joint','l_elbow_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])      


    targets = [[-0.15070158,  0.47726995,  1.56714123],
                [-0.36535318,  0.11249, 1.08326675],
                [-0.56491217,  0.011443, 1.2922572 ],
                [-1.07012697,  0.81909669,  0.47344636],
                [-1.11050811,  0.97000718,  1.31087581]]

    doflimits = robot.GetActiveDOFLimits() #make sure q doesn't go past these limits
    q = numpy.zeros((1,robot.GetActiveDOF())) #start at this configuration
    with env:
        start = time.clock()
	handles = [] #graphics handles for plotting
        SetActiveDOFValuesNPMatrix(robot,q)

        ### YOUR CODE HERE ###

    robot.SetActiveDOFValues([0,0,0,0,0,0,0]);
    robot.GetController().SetDesired(robot.GetDOFValues());

    target = targets[4] ###pick your target here
    #draw the target point in blue
    handles.append(env.plot3(points=array(target), pointsize=15.0, colors=array((0,0,1)) ))

    test_dof_values = [0.24,1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]
    #GetTranslationJacobian(robot,test)
    result = GetEETransform(robot)
    # print 'GetEETransform(robot) = ', result
    # print 'type(result) =',type(result)

    q_config_ary = robot.GetActiveDOFValues()

    searching = 1
    threshold = 0.1
    alpha = 0.5
    while searching>0:

        tgt_xyz_ary = array(target)
        ee_tsf_cur_ary = GetEETransform(robot)
        ee_xyz_cur_ary = [ee_tsf_cur_ary[0][3], ee_tsf_cur_ary[1][3], ee_tsf_cur_ary[2][3]]

        dta_xyz_ary = tgt_xyz_ary - ee_xyz_cur_ary

        if linalg.norm(dta_xyz_ary) < threshold:

            ans_config_ary = q_config_ary
            searching = -1

            print 'Found !!! '
            print 'ans_config_ary = ',ans_config_ary

        j_list = GetTranslationJacobian(robot,jointnames)
        psd_ivs_j_mat = GetJpinv(j_list)
        dta_xyz_ary_t = matrix(dta_xyz_ary).T
        q_increment_mat = psd_ivs_j_mat*dta_xyz_ary_t
        q_increment_mat_t = q_increment_mat.T
        q_increment_lst = q_increment_mat_t.tolist()
        print 'q_increment_lst[0] = ',q_increment_lst[0]
        q_increment_ary = array(q_increment_lst[0])
        print 'q_increment_ary = ', q_increment_ary
        if linalg.norm(q_increment_ary)>alpha:
            q_increment_ary = (alpha/linalg.norm(q_increment_ary))*q_increment_ary


        q_config_ary = q_config_ary + q_increment_ary

        print 'q_config_ary =', q_config_ary

        config_up_limit_list = [2.13539289e+00, 1.29629967e+00, 3.74999698e+00, -1.50000054e-01, 1.00000000e+04,  -1.00000036e-01, 1.00000000e+04]

        config_down_limit_list = [-5.64601796e-01, -3.53600216e-01, -6.50000756e-01, -2.12130808e+00,  -1.00000000e+04,  -2.00000770e+00, -1.00000000e+04]

        for i in range(0, 7):
            if q_increment_ary[i] > config_up_limit_list[i]:
                q_increment_ary[i] = config_up_limit_list[i]
            elif q_increment_ary[i] < config_down_limit_list[i]:
                q_increment_ary[i] = config_down_limit_list[i]

        print 'now q_config_ary = ',q_config_ary

        robot.SetActiveDOFValues(q_config_ary.tolist());
        robot.GetController().SetDesired(robot.GetDOFValues());

        searching = searching + 1
        print 'searching = ',searching

        if searching == 10000:
            searching = -1



    joint_result = GetJointAxis(robot, jointnames[0])



    jcb_lst = GetTranslationJacobian(robot, jointnames)
    psd_jcb_mat = GetJpinv(jcb_lst)

    print 'Pseudo-Inverse Jaccobian =',psd_jcb_mat



    # for joint_name in jointnames:
    #     print joint_name
    #
    #     joint_axis = GetJointAxis(robot,joint_name)
    #     print 'joint_axis = ',joint_axis
    #     print 'type(joint_axis) =',type(joint_axis)
    #
    #     joint_position = GetJointPosition(robot, joint_name)
    #     print 'joint_position = ', joint_position
    #     print 'type(joint_position) =',type(joint_position)
    #
    #     print ''

    #print 'robot.GetActiveDOFLimits() = ', robot.GetActiveDOFLimits()

    '''
    robot.GetActiveDOFLimits() =  (array([ -5.64601796e-01,  -3.53600216e-01,  -6.50000756e-01,
        -2.12130808e+00,  -1.00000000e+04,  -2.00000770e+00, -1.00000000e+04]),
        array([  2.13539289e+00,   1.29629967e+00,   3.74999698e+00, -1.50000054e-01,
        1.00000000e+04,  -1.00000036e-01, 1.00000000e+04]))

    '''

    ### YOUR CODE HERE ###

    robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

    raw_input("Press enter to exit...")
    env.Destroy()
