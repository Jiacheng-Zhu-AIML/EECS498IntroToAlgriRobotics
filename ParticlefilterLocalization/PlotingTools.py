'''
Umich EECS498 Introduction to Algorithmic Robotics Fall 2017
Instructor: Dmitry Berenson
GSI: Kevin French

Openrave environment for demonstration

Ploting and painting tools

Xiaoke Wang, Jiacheng Zhu
'''

import numpy as np
import openravepy
from numpy import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *



def PaintPoint(env, position_list, r=1, g=0, b=0, size=10):

    '''
    General Plot points in openrave envionment
    :param position_list: [ a[ x, y , ...], b[x, y,...]
    :param env: env
    :param r:
    :param g:
    :param b:
    :param size:
    :return:
    '''

    points_list =[]

    for one in position_list:
        x = one[0]
        y = one[1]
        z = 2.0
        points_list.append(env.plot3(points=array(((x, y, z), (x, y, z))),
                                    pointsize=size,
                                    colors=array(((r, g, b), (r, g, b)))))
    return points_list

def PaintPositionBox(env, x, y, theta=0, r=1, g=0, b=0, line_width=5):

    '''

    :param x: position_x_float
    :param y: position_y_float
    :param theta: position_theta_float
    :param env: env_object
    :param r: red_0-1_float
    :param g: green_0-1_float
    :param b: blue_0-1_float
    :param line_width: 0.15-7_float
    :return: list : [one_box_paint_object]
    '''

    lines_list = []
    delta_x = 0.2
    delta_y = 0.3
    z = 2.0

    pt_1 = matrix([[delta_x], [delta_y], [z], [1]])
    pt_2 = matrix([[delta_x], [-delta_y], [z], [1]])
    pt_3 = matrix([[-delta_x], [-delta_y], [z], [1]])
    pt_4 = matrix([[-delta_x], [delta_y], [z], [1]])

    rotation_matrix = matrix([[cos(theta), -sin(theta), 0, 0],
                              [sin(theta),  cos(theta), 0, 0],
                              [         0,           0, 1, 0],
                              [         0,           0, 0, 1]])

    translation_matrix = matrix([[1, 0, 0, x],
                                 [0, 1, 0, y],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])

    transform_matrix = translation_matrix*rotation_matrix

    n_pt_1 = transform_matrix*pt_1
    n_pt_2 = transform_matrix*pt_2
    n_pt_3 = transform_matrix*pt_3
    n_pt_4 = transform_matrix*pt_4

    lines_list.append(env.drawlinestrip(points=array(((n_pt_1[0],n_pt_1[1],z),(n_pt_2[0],n_pt_2[1],z),
                                                      (n_pt_3[0],n_pt_3[1],z),(n_pt_4[0],n_pt_4[1],z),
                                                      (n_pt_1[0],n_pt_1[1],z))),
                                   linewidth=line_width,
                                   colors=array(((r,g,b),(r,g,b),(r,g,b),(r,g,b),(r,g,b)))))

    return lines_list

def PaintPositionBoxList(env, positions_list, r=1, g=0, b=0, line_width=5):

    result_list = []
    for one in positions_list:
        list_temp = PaintPositionBox(env, one[0], one[1])
        result_list = result_list + list_temp

    return result_list

def PaintPositionBoxList_theta(env, positions_list, r=1, g=0, b=0, line_width=5):

    result_list = []
    for one in positions_list:
        list_temp = PaintPositionBox(env, one[0], one[1], one[2])
        result_list = result_list + list_temp

    return result_list


def UpdateTrajectoryLine(env,paint_list, last_position, this_position, r=1, g=0, b=0, line_width =5):
    z = 2
    line_object = env.drawlinestrip(points=array(((last_position[0], last_position[1], z),
                                                  (this_position[0], this_position[1], z))),
                                   linewidth=line_width,
                                   colors=array(((r,g,b),(r,g,b))))

    paint_list.append(line_object)

    return paint_list
