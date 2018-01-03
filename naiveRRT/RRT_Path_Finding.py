'''
EECS498 HW03 RRT
JiachengZhu jiachzhu@umich,edu

'Brutal Memorize RRT Tree'
Some times there will be openrave error, just try again please. The algorithm does work for most of the times

'''

import numpy as np
import random
import types
import time
from RRT_Tree_Customed import Branch
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def RRTPathFindingFunc(init_config,goal_config,env,increment_input = 0.05, criteria = 0.05):

    list = []
    Tree = {}
    Tree[0] = Branch([ np.array(init_config),np.array(init_config)],0,0,init_config)
    finding = 1
    small_test = 1
    min_dist = 1000

    print 'init_config =', init_config
    compute_time = time.clock()
    while finding >0:

        print 'iterate:finding = ',finding
        print 'iterate:small_test =', small_test
        c0 = random.uniform(-0.565, 2.140)
        c1 = random.uniform(-0.354, 1.296)
        c2 = random.uniform(-2.121, -0.150)
        c3 = random.uniform(-0.650, 3.750)
        c4 = random.uniform(-np.pi, np.pi)
        c5 = random.uniform(-2.0, -0.1)
        try_config = [c0,c1,c2,c3,c4,c5]

        rem = small_test % 10
        if rem == 1:
            try_config = goal_config
        nearest_result = FindNearest(Tree,try_config)
        nearest_branch_id = nearest_result[0]
        nearest_branch = nearest_result[1]
        nearest_configofbranch = nearest_result[2]
        grow_tree = GrowNewTreeEndConfig(try_config,nearest_configofbranch,0.02,env)
        new_config = grow_tree[0]
        print ' '
        small_test = small_test + 1

        if grow_tree[1] == True:

            new_branch = Branch([nearest_configofbranch,new_config],finding,nearest_branch_id,try_config)
            new_branch.parentbranch = nearest_branch
            Tree[finding] = new_branch
            finding = finding + 1
            branch_length = ConfigPointsDistance(nearest_configofbranch, new_config)

            if ConfigPointsDistance(goal_config,new_config) < min_dist:
                min_dist = ConfigPointsDistance(goal_config,new_config)
                min_branch = new_branch

            now_time = time.clock() - compute_time
            print 'Now the smallest dist is ', min_dist,'Length of this branch is', branch_length,''
            print '**********************************************************************************************************Now the smallest dist is  =',min_dist,'time used is:',now_time
            print 'new_branch.parent_branch_id =',nearest_branch_id
            print 'length_test,ConfigPointsDistance(new_config,try_config) ', ConfigPointsDistance(new_config,try_config)
            print ''

            if ConfigPointsDistance(goal_config,new_config)< criteria:
                finding = -1
                print 'Found !'

        if small_test%1000 == 999:
            new_key_branch = None
            for key in Tree:
                if Tree[key].tested == False:
                    Tree[key].tested == True
                    test_branch = Tree[key]
                    root_config = NewBranchConfigTest(test_branch,goal_config,0.02)[1]
                    grow_key_tree = GrowNewTreeEndConfig(goal_config,root_config,0.02,env)
                    dist = ConfigPointsDistance(root_config,goal_config)
                    if grow_key_tree[1] == True and  ConfigPointsDistance(grow_key_tree[0], goal_config)<=criteria:
                        new_key_branch = Branch([root_config, grow_key_tree[0]],finding + 1,key,goal_config)
                        new_key_branch.parentbranch = Tree[key]
                        for skey in Tree:
                            Tree[skey].tested =True

                        break

            if new_key_branch is not None:
                print 'Found Key Branch!!!!!!!!!!!!!!'

                Tree[finding+1] = new_key_branch
                min_branch = new_key_branch
                finding = -1

        if finding == 10000 or small_test == 3000:
            finding = -1
            list.insert(0, min_branch.config_b)

    while min_branch.parentbranch != None:
        if isinstance(min_branch.config_a,types.ListType):
            list.insert(0,min_branch.config_a)
        else:
            config_a = min_branch.config_a
            config_b = min_branch.config_b
            connect_list = ConnectConfig(config_a, config_b,env,0.005)
            #list.insert(0,min_branch.config_a.tolist())
            list = connect_list + list

    # for one in list:
    #     config_a = one:
    #     config_b = list[list.index(one) + 1]
    #     connnect = ConnectConfig(config_a, config_b, env, 0.005)


        min_branch = min_branch.parentbranch
    list.append(goal_config)

    for key in Tree:
        print 'key =', key
        print 'Tree[key].brachid =',Tree[key].branchid
        print 'Tree[key].config_a = ',Tree[key].config_a
        print 'Tree[key].config_b = ', Tree[key].config_b
        print 'Tree[key].parentid = ',Tree[key].parentid
        print 'Tree[key].targetconfig = ',Tree[key].targetconfig
        print 'BranchLength =',ConfigPointsDistance(Tree[key].config_a,Tree[key].config_b)
    ii = 0
    for ii in range(len(list) - 1):
        result_list = []
        connect = ConnectConfig(list[ii],list[ii + 1],env, 0.03)
        if connect[1] == True:
            result_list = list[:ii] + connect[0] + list[ii:]

    return [result_list,Tree]

def PathSmooth(path_list,increment,env):
    new_path = []
    result_list = path_list
    list_length = len(path_list)
    path_length = 0;
    for i in range(list_length):
        for j in range(list_length):
            #test path_list[i] and path_list[j]
            if (path_list[i] is not None) and (path_list[j] is not None) and (i is not j):
                #testing
                test_result = GrowNewTreeEndConfig(path_list[i], path_list[j], increment, env)
                if test_result[1] == True and ConfigPointsDistance(path_list[i], test_result[0]) <= increment:
                    for ii in range(min(i,j)+1,max(i,j)):
                        result_list[ii] = None
    do = 0
    while do >= 0 and do < len(result_list):
        if result_list[do] == None:
            del result_list[do]
        else:
            do = do + 1
    return result_list

def PathSmoothAdvanced(path_list,increment,env):

    ite = 1
    result_list = path_list[:]
    while ite < 150:
        a = int(random.uniform(0,len(result_list)))
        b = int(random.uniform(0,len(result_list)))
        if a == b:
            break
        else:
            n = min(a,b)
            m = max(a,b)
            ite = ite + 1
            connect = ConnectConfig(result_list[n], result_list[m], env, increment)
            if connect[1] == True:
                result_list = path_list[:n] + connect[0] + path_list[m:]

    return result_list

def ConnectConfig(config_a,config_b,env,increment = 0.01):
    result_config_list = []
    grow_path_vector = ConfigPointAddMinus(config_b, np.multiply(-1, config_a))
    if ConfigPointsDistance(grow_path_vector) == 0:
        coe = 0
    else:
        coe = increment / ConfigPointsDistance(grow_path_vector)

    increment_vector = np.multiply(coe,grow_path_vector)
    testing = 1
    test_config = config_a
    useless = False
    while testing >= 0:
        test_config = ConfigPointAddMinus(test_config, increment_vector)
        testing = testing + 1
        result_config_list.append(test_config)

        if DetectConfigLimit(test_config) or DetectConifgCollsion(test_config, env) or ( (testing-1)*ConfigPointsDistance(increment_vector) >= ConfigPointsDistance(grow_path_vector)):
            if testing == 2:
                useful = False
            else:
                useful = True
            result_config = ConfigPointAddMinus(test_config, np.multiply(-1, increment_vector))
            result_config_list.pop()
            testing = -10
    return [result_config_list, useful]

def ConfigPointsDistance(config_A,config_B=[0,0,0,0,0,0]):
    sum = 0
    for i in range(0,len(config_A)):
        if i == 0:
            sum = sum + (config_A[i] - config_B[i]) ** 2
        elif i == 1:
            sum = sum + (config_A[i] - config_B[i]) ** 2
        elif i == 2:
            sum = sum + ((config_A[i] - config_B[i]) ** 2)/5
        elif i == 3 :
            sum = sum + ((config_A[i] - config_B[i]) ** 2)/20
        elif i == 4:
            sum = sum + (min(abs(config_A[i] - config_B[i]), 2*np.pi - abs(config_A[i]-config_B[i]))**2)/5000
        elif i == 5 :
            sum = sum + ((config_A[i] - config_B[i])**2)/5000
    ans = sum**0.5
    return ans

def ConfigPointAddMinus(config_A,config_B):
    ans = []
    for i in range(0,len(config_A)):
        if i == 4:
            temp = config_A[i] + config_B[i]
            if temp >= float(np.pi):
                temp = temp - 2*np.pi
            elif temp <= float(-np.pi):
                temp = temp + 2*np.pi
            ans.append(temp)
        else:
            ans.append(config_A[i] + config_B[i])
    return ans


def NewBranchConfigTest(Branch,config,increment=0.02):
    branch_path_vector = ConfigPointAddMinus(Branch.config_b, np.multiply(-1,Branch.config_a))
    if ConfigPointsDistance(branch_path_vector) == 0:
        coe = 0
    else:
        coe = increment/ConfigPointsDistance(branch_path_vector)
    increment_vector = np.multiply(coe,branch_path_vector)
    adding = 0
    dist = 100
    test_config = Branch.config_a
    while adding >= 0:
        test_config = ConfigPointAddMinus(test_config,increment_vector)
        if ConfigPointsDistance(config,test_config)<dist:
            dist = ConfigPointsDistance(config,test_config)
            config_h = test_config
        adding = adding + 1
        if ConfigPointsDistance(test_config) >= ConfigPointsDistance(branch_path_vector):
            adding = -1
    return [dist,config_h]


def FindNearest(treedict,test_config):
    result_id = None
    temp_dist = 10000
    result_branch = None
    result_config_h = None
    for key in treedict:
        test_id = key
        test_branch = treedict[key]
        branchconfigtest_result = NewBranchConfigTest(test_branch,test_config)
        if branchconfigtest_result[0]<temp_dist:
            result_id = test_id
            temp_dist = NewBranchConfigTest(test_branch,test_config)[0]
            result_branch = test_branch
            result_config_h = branchconfigtest_result[1]

    return [result_id, result_branch, result_config_h]

def GrowNewTreeEndConfig(config,root_config,increment,env):

    grow_path_vector = ConfigPointAddMinus(config, np.multiply(-1, root_config))
    if ConfigPointsDistance(grow_path_vector) == 0:
        coe = 0
    else:
        coe = increment / ConfigPointsDistance(grow_path_vector)

    increment_vector = np.multiply(coe,grow_path_vector)
    testing = 1
    test_config = root_config
    while testing >= 0:
        test_config = ConfigPointAddMinus(test_config, increment_vector)
        testing = testing + 1

        if DetectConfigLimit(test_config) or DetectConifgCollsion(test_config, env) or ( (testing-1)*ConfigPointsDistance(increment_vector) >= ConfigPointsDistance(grow_path_vector)):
            if testing == 2:
                useful = False
            else:
                useful = True
            result_config = ConfigPointAddMinus(test_config, np.multiply(-1, increment_vector))
            testing = -10
    return [result_config,useful]


def DetectConifgCollsion(config,env):

    target_config = config
    robot = env.GetRobots()[0]
    jointnames = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_elbow_flex_joint', 'l_upper_arm_roll_joint',
                  'l_forearm_roll_joint', 'l_wrist_flex_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
    startconfig = target_config
    robot.SetActiveDOFValues(startconfig);
    robot.GetController().SetDesired(robot.GetDOFValues());

    result = env.CheckCollision(robot)
    if DetectConfigLimit(config):
        result = True

    return result

def DetectConfigLimit(config):
    out_of_limit = True
    if -0.565<config[0] and config[0]<2.135:
        if -3.534<config[1] and config[1]<1.286:
            if -2.122<config[2] and config[2]<-0.151:
                if -0.651<config[3] and config[3]<3.749:
                    if -5.0<config[4] and config[4]<5.0:
                        if -2.0<config[5] and config[5]<-0.1:
                            out_of_limit = False

    return out_of_limit


def DrawPath(path_list,env, r=0, g=0, b=1,point_size = 10.0):
    robot = env.GetRobots()[0]
    points_list = []
    for one in path_list:
        jointnames = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_elbow_flex_joint', 'l_upper_arm_roll_joint',
                      'l_forearm_roll_joint', 'l_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues(one);
        robot.GetController().SetDesired(robot.GetDOFValues());

        #link_tf = robot.GetLink('l_gripper_l_finger_link').GetTransform()
        link_tf = robot.GetActiveManipulator().GetEndEffectorTransform()
        x = link_tf[0][3]
        y = link_tf[1][3]
        z = link_tf[2][3]
        points_list.append(env.plot3(points=array(((x, y, z), (x, y, z))),
                                     pointsize=point_size,
                                     colors=array(((r, g, b), (r, g, b)))))

    return points_list


'''
The area of this configs
(array([ -5.64601796e-01,  -3.53600216e-01,  -2.12130808e+00,
        -6.50000756e-01,  -1.00000000e+04,  -2.00000770e+00]),

array([  2.13539289e+00,   1.29629967e+00,  -1.50000054e-01,
         3.74999698e+00,   1.00000000e+04,  -1.00000036e-01]))

<link:l_gripper_l_finger_link (48), parent=PR2>
<link:l_gripper_l_finger_tip_link (49), parent=PR2>
<link:l_gripper_motor_slider_link (50), parent=PR2>
<link:l_gripper_motor_screw_link (51), parent=PR2>
<link:l_gripper_led_frame (52), parent=PR2>
<link:l_gripper_motor_accelerometer_link (53), parent=PR2>
<link:l_gripper_r_finger_link (54), parent=PR2>
<link:l_gripper_r_finger_tip_link (55), parent=PR2>
<link:l_gripper_l_finger_tip_frame (56), parent=PR2>
<link:l_gripper_tool_frame (57), parent=PR2>
RaveGetEnvironment(1).GetKinBody('PR2').GetLink('l_gripper_l_finger_tip_link')



'''
