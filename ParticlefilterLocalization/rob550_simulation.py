

# -*- coding: utf-8 -*-
import time
import openravepy
from PlotingTools import *
import matplotlib.pyplot as plt
from MoveFunction import *
from SensorFunction import *
from PlotingTools import *
from ParticleFilterFunction import *

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

def plot_demo(true_locations, sensed_locations, filtered_locations):
    plt.figure(1)

    # circle1 = plt.Circle((target_location[0],target_location[1]), criterion, color='r')
    # plt.gcf().gca().add_artist(circle1)

    true_locations_x = []
    true_locations_y = []
    for i in range(0, len(true_locations)):
        true_locations_x.append(true_locations[i][0])
        true_locations_y.append(true_locations[i][1])
    plt.plot(true_locations_x, true_locations_y, 'k.-', alpha=0.9)

    sensed_locations_x = []
    sensed_locations_y = []
    for i in range(0, len(sensed_locations)):
        sensed_locations_x.append(sensed_locations[i][0])
        sensed_locations_y.append(sensed_locations[i][1])
    plt.plot(sensed_locations_x, sensed_locations_y, 'ro', alpha=0.5, label='sensed trajoctory')

    filtered_locations_x = []
    filtered_locations_y = []
    for i in range(0, len(filtered_locations)):
        filtered_locations_x.append(filtered_locations[i][0])
        filtered_locations_y.append(filtered_locations[i][1])
    plt.plot(filtered_locations_x, filtered_locations_y, 'gx-', alpha=0.5, label='filtered trajoctory')
    plt.legend(loc='upper left')

    true_locations_x_array = np.array(true_locations_x)
    true_locations_y_array = np.array(true_locations_y)
    sensed_locations_x_array = np.array(sensed_locations_x)
    sensed_locations_y_array = np.array(sensed_locations_y)
    filtered_locations_x_array = np.array(filtered_locations_x)
    filtered_locations_y_array = np.array(filtered_locations_y)
    distance_f = np.sqrt((true_locations_x_array - filtered_locations_x_array) ** 2 + (
    true_locations_y_array - filtered_locations_y_array) ** 2)
    distance_s = np.sqrt((true_locations_x_array - sensed_locations_x_array) ** 2 + (
    true_locations_y_array - sensed_locations_y_array) ** 2)

    plt.figure(2)
    plt.plot(range(0, distance_f.shape[0]), list(distance_f), 'g.-', range(0, distance_s.shape[0]),
             list(distance_s), 'r.-')
    plt.xlabel('iterations')
    plt.ylabel('error(distance)')
    plt.legend(('filter error', 'sensor error'),
               loc='upper right')
    print 'average of filter distance error:', sum(distance_f) / len(distance_f)
    print 'average of sensor distance error', sum(distance_s) / len(distance_s)
    plt.show()
        # plt.show()


def plot_error(true_locations, sensed_locations, filtered_locations):
    true_locations_x = []
    true_locations_y = []
    for i in range(0, len(true_locations)):
        true_locations_x.append(true_locations[i][0])
        true_locations_y.append(true_locations[i][1])
    # plt.plot(true_locations_x,true_locations_y,'k.-',alpha=0.9)

    sensed_locations_x = []
    sensed_locations_y = []
    for i in range(0, len(sensed_locations)):
        sensed_locations_x.append(sensed_locations[i][0])
        sensed_locations_y.append(sensed_locations[i][1])
    # plt.plot(sensed_locations_x,sensed_locations_y,'ro',alpha=0.5,label = 'sensed trajoctory')

    filtered_locations_x = []
    filtered_locations_y = []
    for i in range(0, len(filtered_locations)):
        filtered_locations_x.append(filtered_locations[i][0])
        filtered_locations_y.append(filtered_locations[i][1])
    # plt.plot(filtered_locations_x,filtered_locations_y,'bx-',alpha=0.5,label = 'filtered trajoctory')
    # plt.legend(loc='upper left')

    true_locations_x_array = np.array(true_locations_x)
    true_locations_y_array = np.array(true_locations_y)
    sensed_locations_x_array = np.array(sensed_locations_x)
    sensed_locations_y_array = np.array(sensed_locations_y)
    filtered_locations_x_array = np.array(filtered_locations_x)
    filtered_locations_y_array = np.array(filtered_locations_y)
    distance_f = np.sqrt((true_locations_x_array - filtered_locations_x_array) ** 2 + (
    true_locations_y_array - filtered_locations_y_array) ** 2)
    distance_s = np.sqrt((true_locations_x_array - sensed_locations_x_array) ** 2 + (
    true_locations_y_array - sensed_locations_y_array) ** 2)

    plt.figure(2)
    plt.plot(range(0, distance_f.shape[0]), list(distance_f), 'g.-', range(0, distance_s.shape[0]), list(distance_s),
             'r.-')
    plt.xlabel('iterations')
    plt.ylabel('error(distance)')
    plt.legend(('filter error', 'sensor error'),
               loc='upper right')
    # label = 'average of filter distance error:'+'\n'+'average of sensor distance error'
    # tx = 2
    # ty = 0.9
    # plt.text(tx,ty,label_f1,fontsize=15,verticalalignment="top",horizontalalignment="right")
    print 'average of filter distance error:', sum(distance_f) / len(distance_f)
    print 'average of sensor distance error', sum(distance_s) / len(distance_s)
    plt.show()
    # distance = np.sqrt((true_locations[i][0]-target_location[0])**2+(true_locations[i][1]-target_location[1])**2)
    return distance_f, distance_s  # ,criterion


if __name__ == "__main__":

    #global env
    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()
    env.Load('map_folder/PR2_Maze_hexagon_test.env.xml')
    x_range = (-10, 10)
    y_range = (-8.66, 8.66)
    time.sleep(0.1)

    robot = env.GetRobots()[0]
    tuckarms(env, robot);

    with env:
        robot.SetActiveDOFs([], DOFAffine.X | DOFAffine.Y | DOFAffine.RotationAxis, [0, 0, 1])

    '''
    Initialize
    '''
    pr2_motion = RobotMotion(env, robot)
    pr2_motion.working = True

    true_location_list = [pr2_motion.TrueLocation()]
    sensed_location_list = [pr2_motion.TrueLocation()]
    filtered_location_list = [pr2_motion.TrueLocation()]
    head_list = [pr2_motion.heading]
    result_location_list = [pr2_motion.TrueLocation()]

    '''
    Initialize ParticleFilter
    '''
    pr2_particles = Particles(env, robot)
    pr2_particles.InitializeParticelsCollision(x_range, y_range, 50, 3)

    particles_temp_paint_list = PaintPoint(env, pr2_particles.particles)
    time.sleep(5)
    particles_temp_paint_list = []


    '''
    Start pseudo real-time open-loop simulation
    [firstpath[headvector[x,y],steps,steplength], secondpath[...],...]
    '''
    close_loop_plan = [[5, -5],[7.5, 0], [5, 7.5]]#, [-1, -2], [0, 0], [3, -2], [4, 2]]
    open_loop_plan = [[[1, 1], 10, 1, 0]] #, [[1, 0], 10, 0.2], [[1, 1], 40, 0.2]]
    pr2_motion.close_loop_target_list = close_loop_plan
    pr2_motion.open_loop_plan_list = open_loop_plan

    true_loaction_paint_list = []
    sensed_location_paint_list = []
    filtered_location_paint_list = []
    true_location_line_paint_list = []
    filter_location_line_list = []

    sigma = 1
    step = 0.5
    sigma_s = 1.6
    sigma_m = 0.01
    # sigma_s = 5
    # sigma_m = 0.9
    now_position = pr2_motion.TrueLocation()  # the first step uses true location
    now_heading = pr2_motion.heading
    # while pr2_motion.working:

    #pr2_motion.working = 0

    while pr2_motion.working:
        '''
        fixed heading
        Move one step in the plan
        '''
        pr2_motion.MoveTorwardTarget(now_position, 1, step, sigma_m)
        #pr2_motion.MoveAsPlaned()

        true_position = pr2_motion.TrueLocation()
        true_location_line_paint_list = UpdateTrajectoryLine(env, true_location_line_paint_list,true_location_list[-1], true_position,0,0,0,10)
        true_location_list.append(true_position)  # get now true loaction

        if true_loaction_paint_list:
            true_loaction_paint_list.pop()


        true_loaction_paint_list.extend(PaintPositionBox(env, true_position[0], true_position[1], 0,0,0,0))




        '''
        Get Heading in this step
        '''
        now_heading = pr2_motion.heading #get now heading
        head_list.append(now_heading) # Record this heading

        '''
        Sensor
        '''
        #if sensed_location_paint_list:
            #sensed_location_paint_list.pop()

        '''
        Use sensor
        Paint sensor
        Record sensor
        '''
        sensed_location, sigmahhh = GaussianSensor(env, robot, sigma_s)  # get sensed location

        sonar_result = SonarSensorRobot(env, robot)


        # if sensed_location_paint_list:
        #    sensed_location_paint_list.pop()

        #sensed_location_paint_list.extend(PaintPositionBox(env, sensed_location[0], sensed_location[1], 0, 1, 0, 0))
        sensed_location_list.append(sensed_location)

        '''
        Particle Filter
        Predict the particles according to the head
        '''
        pr2_particles.PredictMove(now_heading)

        if particles_temp_paint_list:
            particles_temp_paint_list = []

        particles_temp_paint_list = PaintPoint(env, pr2_particles.particles, 0,1,0,5)
        #time.sleep(5)

        '''
        Reweighting the samples and resample
        Paint the particles
        '''
        #pr2_particles.UpdateWeight(sensed_location, EcliudWeighting)

        pr2_particles.UpdateWeightSonar(sonar_result, SonarWeighting)

        pr2_particles.resample_gaussian_sigma_list = [0.8, 0.8]
        pr2_particles.Resample(GaussianResample, 0.4)


        if particles_temp_paint_list:
            particles_temp_paint_list = []

        particles_temp_paint_list = PaintPoint(env, pr2_particles.particles, 0,1,0,5)
        #time.sleep(5)

        '''
        Return results
        Paint results
        '''
        #result_position = pr2_particles.Estimate_XY()
        filtered_location = pr2_particles.EstimateByWeight()
        filter_location_line_list = UpdateTrajectoryLine(env, filter_location_line_list, filtered_location_list[-1],
                                                         filtered_location, 0, 1, 0, 10)

        filtered_location_list.append(filtered_location)


        if filtered_location_paint_list:
            filtered_location_paint_list.pop()

        filtered_location_paint_list.extend(PaintPositionBox(env, filtered_location[0], filtered_location[1], 0, 0, 1, 0))

        now_position = filtered_location

        print 'pr2_motion.moving =', pr2_motion.moving
        '''
        End of this step

        '''

    print 'true_location_list =', true_location_list
    print 'sensed_location_list =', sensed_location_list
    print 'filtered_location_list =', filtered_location_list
    print 'head_list =', head_list
    plot_demo(true_location_list, sensed_location_list, filtered_location_list)

    x1 = 0
    x2 = 0
    # i = 0
    flag = 1
    for i in range(len(true_location_list)):
        if flag == 1:
            if true_location_list[i][1] > -8:
                x1 = i
                flag = 2
        if flag == 2:
            if true_location_list[i][1] > 8:
                x2 = i
                flag = 3
                # if flag == 1:
                #     if true_location_list[i][1]<-8:
                #         x1 = i
                #         flag = 2
                # i = i+1
    print x1, x2

    list1t = true_location_list[0:x1]
    list2t = true_location_list[x1:x2]
    list3t = true_location_list[x2:]

    list1s = sensed_location_list[0:x1]
    list2s = sensed_location_list[x1:x2]
    list3s = sensed_location_list[x2:]

    list1f = filtered_location_list[0:x1]
    list2f = filtered_location_list[x1:x2]
    list3f = filtered_location_list[x2:]

    plot_error(list1t, list1s, list1f)
    plot_error(list2t, list2s, list2f)
    plot_error(list3t, list3s, list3f)

    raw_input("Press enter to exit...")


    raw_input("Press enter to exit...")
