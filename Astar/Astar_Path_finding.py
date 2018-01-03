'''
EECS498 HW03 A-star
JiachengZhu jiachzhu@umich,edu

'A-Star Path Finding Algorithm '
Use 8-connected as default
Uncomment line to use 4-connected neighbors

Need to be modified
'''


from Queue import PriorityQueue
import numpy as np
from PriorityQueue_Customed import Node
import openravepy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


def AstarPathFinding(delta_x, delta_y, delta_theta, start_position, goal_position,env):
    print 'Using AstarPathFinding'
    path = []
    detected_list = []
    collision_list = []
    Queue_p = PriorityQueue()

    now_position = start_position
    now_node = Node(start_position,0,0,0)

    i = 0
    keepsearching = 1
    while keepsearching >0:
        for onex in [now_position[0]-delta_x, now_position[0], now_position[0] + delta_x]:
            for oney in [now_position[1]-delta_y, now_position[1], now_position[1] + delta_y]:
                for onetheta in [now_position[2]-delta_theta, now_position[2], now_position[2] + delta_theta]:
                    #if onex == now_position[0] or oney == now_position[1]:#Uncomment this row to enable 4-connected
                    if keepsearching == 1:#Uncomment this to Enable 8-connected
                        #print 'In the small loop, [onex,oney,onetheta] =',[onex,oney,onetheta]
                        if DetectPointCollsion(env,[onex,oney,onetheta]) == False:
                            if [onex,oney,onetheta] not in detected_list:

                                print 'in the AstarPathFinding,i=',i
                                i=i+1

                                detected_list.append([onex,oney,onetheta])
                                step_cost =  GActionCost([onex,oney,onetheta], now_node.position)
                                heuristic =  GActionCost([onex,oney,onetheta],goal_position)
                                this_cost = now_node.distacne + step_cost
                                priority = this_cost + heuristic
                                temp = Node([onex,oney,onetheta],i,now_node.id,this_cost)
                                temp.parentnode = now_node

                                Queue_p.put((priority,temp))
                        else:
                            collision_list.append([onex,oney,onetheta])


        #emmmmm
        tempobj = Queue_p.get()
        now_node = tempobj[1]
        now_position = now_node.position

        if GActionCost(now_position,goal_position)<0.1:
            keepsearching = -1
            final_cost = now_node.distacne
            adding = 1
            while now_node!=None:
                path.insert(0,now_node.position)
                now_node = now_node.parentnode
            print len(path)

    return [path,detected_list,final_cost, collision_list]

def GActionCost(point1,point2):
    pt1 = (point1[0]-point2[0])**2 + (point1[1] - point2[1])
    pt2 = min(abs(point1[2] - point2[2]),2*np.pi - abs(point1[2] - point2[2]))
    ans = (pt1**2 + pt2**2)**0.5

    return ans

def DetectPointCollsion(env,position):
    #print 'Using DetectPointCollision'
    target_robot = position
    robot = env.GetRobots()[0]
    robot_position = robot.GetTransform()

    Tz = matrixFromAxisAngle([0, 0, 0])
    Tz[0][3] = target_robot[0] - robot_position[0][3]
    Tz[1][3] = target_robot[1] - robot_position[1][3]
        # Tz[2][3] = target_robot[2] - robot_position[2][3]

    robot.SetTransform(np.dot(Tz, robot.GetTransform()))
    #print 'in collision func, Now robot.GetTransform() = ', robot.GetTransform()
    return env.CheckCollision(robot)

def PaintPoint(position_list,env,r,g,b,size):

    points_list =[]

    for one in position_list:
        x = one[0]
        y = one[1]
        z = 0.5
        points_list.append(env.plot3(points=array(((x, y, z), (x, y, z))),
                                    pointsize=size,
                                    colors=array(((r, g, b), (r, g, b)))))
    return points_list