'''
EECS498
JiachengZhu jiachzhu@umich,edu

'Bayes Filter Localization'

Need to be modified
'''

import matplotlib.pyplot as plt
import numpy as np
from numpy import *
from matplotlib.ticker import MultipleLocator, FuncFormatter
import random

def Visualize():

    plt.plot(array([0.5, 15.5]), array([0.5, 0.5]), color='black')
    plt.plot(array([0.5, 0.5]), array([0.5, 10.5]), color='black')
    plt.plot(array([0.5, 15.5]), array([10.5, 10.5]), color='black')
    plt.plot(array([15.5, 15.5]), array([0.5, 10.5]), color='black')

    plt.grid()  # == plt.grid(True)
    plt.grid(color='b', linewidth='0.3', linestyle='--')
    ax = plt.gca()
    ax.xaxis.set_major_locator(MultipleLocator(1))
    ax.yaxis.set_major_locator(MultipleLocator(1))
    #ax.invert_yaxis()

    #plt.show()
    return plt


def PlotPath(point_list, color_str, plt, marker_char='o'):

    x_list = []
    y_list = []

    for one in point_list:
        x_list.append(one[0])
        y_list.append(one[1])

    x_list_array = array(x_list)
    y_list_array = array(y_list)

    plt.plot(x_list_array, y_list_array, color_str, marker=marker_char, alpha=0.4)

    return plt

def Simulate():

    plt = Visualize()
    true_location_sublist = [5, 7]
    true_location_sublist_list = [true_location_sublist]
    sensed_location_sublist_list = [true_location_sublist]
    filtered_location_sublist_list = [true_location_sublist]

    head_sublist = GeneratHead()
    initial_state_matrix = zeros([17,12])
    initial_state_matrix[5,7] = 1
    state_this_matrix = initial_state_matrix
    for i in range(30):

        print 'now head sublist is', head_sublist
        move_result = Move(true_location_sublist, head_sublist)
        true_location_sublist_list.append(move_result[0])
        true_location_sublist = move_result[0]
        sensed_location_sublist = Sense(true_location_sublist)
        sensed_location_sublist_list.append(sensed_location_sublist)



        filter_result = Filter(state_this_matrix, head_sublist, sensed_location_sublist)

        state_this_matrix = filter_result[0]
        filtered_location_sublist_list.append(filter_result[1])
        print 'filter_result[1] =', filter_result[1]

        if move_result[1]:
            head_sublist = GeneratHead()

    plt = PlotPath(true_location_sublist_list, 'black', plt)
    plt = PlotPath(sensed_location_sublist_list, 'ro', plt, 'x')
    plt = PlotPath(filtered_location_sublist_list, 'blue', plt)

    print 'true_location_sublist_list =', true_location_sublist_list
    print 'sensed_location_sublist_list =', sensed_location_sublist_list

    print 'len(true_location_sublist_list) =', len(true_location_sublist_list)
    print 'len(sensed_location_sublist_list) =', len(sensed_location_sublist_list)
    print 'len(filtered_location_sublist_list) =', len(filtered_location_sublist_list)

    error_list = []
    for i in range(len(true_location_sublist_list)):

        error=((true_location_sublist_list[i][0]-filtered_location_sublist_list[i][0])**2 + (true_location_sublist_list[i][1]-filtered_location_sublist_list[i][1])**2)**0.5
        error_list.append(error)

    fig2 = plt.figure(2)
    plt.plot(array(range(len(true_location_sublist_list))), array(error_list))

    plt.show()



def Sense(true_location_sublist):

    eight_point_list = []
    sixteen_point_list = []
    for i in [true_location_sublist[0] - 1, true_location_sublist[0], true_location_sublist[0] + 1]:
        for j in [true_location_sublist[1] - 1, true_location_sublist[1], true_location_sublist[1] + 1]:
            if i != true_location_sublist[0] or j != true_location_sublist[1]:
                eight_point_list.append([i, j])


    for i in range(true_location_sublist[0] - 2, true_location_sublist[0] + 3, 1):
        for j in range(true_location_sublist[1] - 2, true_location_sublist[1] + 3, 1):
            if abs(true_location_sublist[0] - i) > 1 or abs(true_location_sublist[1] - j) > 1:
                sixteen_point_list.append([i,j])

    random_number = random.random()
    if random_number < 0.15:
        result = true_location_sublist
    elif random_number < 0.6:
        random_index = random.randint(0, 7)
        result = eight_point_list[random_index]
    else:
        random_index = random.randint(0, 15)
        result = sixteen_point_list[random_index]

    return result


def Filter(last_state_matrix, this_head_sublist, this_sensor_sublist):

    kernel_head_matrix = HeadSublistToKernelmatrix(this_head_sublist)
    #print 'kernel_head_matrix =', kernel_head_matrix
    '''
    P(STATE_t+1|sensor_1:t)= Sigma/state1 P(STATE_t+1|state_t)P(state_t|sensor_t)
    convolution (STATE1(Matrix), move_head_kernel)
    '''
    state_this_temp_matrix = Convolution(last_state_matrix, kernel_head_matrix)
    #print 'state_this_temp_matrix ='
    #print state_this_temp_matrix
    '''
    P(STATE_t+1|sensor_1:t) = a*P(SENSOR_t+1|STATE_t+1) P(STATE_t+1|sensor_1:t)
    '''
    sensor_this_state_matrix = SensorSublistToState(this_sensor_sublist)
    #print 'sensor_this_state_matrix'
    #print sensor_this_state_matrix
    this_state_matrix_temp = multiply(state_this_temp_matrix, sensor_this_state_matrix)
    #print 'this_state_matrix_temp ='
    #print this_state_matrix_temp

    this_state_matrix = NormalizationExceptPad(this_state_matrix_temp)
    #print 'this_state_matrix'
    #print this_state_matrix
    max_num = 0
    max_sublist = []
    for ii in range(1, len(this_state_matrix) - 1):
        for jj in range(1, len(this_state_matrix[0]) - 1):
            if this_state_matrix[ii, jj] > max_num:
                max_num = this_state_matrix[ii,jj]
                max_sublist = [ii,jj]

    return [this_state_matrix, max_sublist]


def Convolution(target_matrix, kernel_matrix):

    #target_matrix is padded
    #kernel must be 3x3
    result_matrix = zeros([len(target_matrix), len(target_matrix[0])])
    for i in range(1, len(target_matrix) - 1):
        for j in range(1, len(target_matrix[0]) - 1):
            for ji in [0, 1, 2]:
                for jj in [0 , 1, 2]:
                    result_matrix[i+ji-1, j+jj-1] = result_matrix[i+ji-1, j+jj-1] + target_matrix[i,j]*kernel_matrix[ji,jj]

    #return result_matrix[1:len(target_matrix)-1, 1:len(target_matrix[0])-1]
    return result_matrix

def HeadSublistToKernelmatrix(head):
    #a transformation
    kernel_matrix = zeros([3, 3])
    kernel_matrix[:] = (0.2/7.0)
    #kernel_matrix[:] = 2
    kernel_matrix[1,1] = 0
    kernel_matrix[1 + head[0], 1 + head[1]] = 0.8
    #kernel_matrix[1 + head[0], 1 + head[1]] = 6

    return kernel_matrix

def SensorSublistToState(sensor_sublist):

    sensor_state_matrix = zeros([17, 12])
    sensor_small_matrix = zeros([5, 5])
    sensor_small_matrix[:] = 0.025
    sensor_small_matrix[1:4, 1:4] = 0.05625
    sensor_small_matrix[2, 2] = 0.15

    if 0:
        sensor_state_matrix[ sensor_sublist[0]-2:sensor_sublist[0]+3, sensor_sublist[1]-2:sensor_sublist[1]+3] = sensor_small_matrix
    else:
        for i in range(sensor_sublist[0]-2, sensor_sublist[0]+3):
            for j in range(sensor_sublist[1]-2, sensor_sublist[1]+3):
                if 0 <= i <= 16 and 0 <= j <= 11:
                    sensor_state_matrix[i, j] = sensor_small_matrix[ i-sensor_sublist[0]-2, j-sensor_sublist[1]-2]

    return sensor_state_matrix

def NormalizationExceptPad(matrix_padded):

    ans_matrix = zeros([len(matrix_padded), len(matrix_padded[0])])
    matrix_except_pad = matrix_padded[1:len(matrix_padded)-1, 1:len(matrix_padded[0]-1)]
    sum_num = matrix_except_pad.sum()
    divide_matrix = zeros([len(matrix_except_pad), len(matrix_except_pad[0])])
    #divide_matrix[:,:] = matrix_except_pad[:,:]
    divide_matrix[:,:] = 1.0/sum_num
    multipy_result_matrix = multiply(matrix_except_pad, divide_matrix)
    ans_matrix[1:len(matrix_padded)-1, 1:len(matrix_padded[0]-1)] = multipy_result_matrix

    return ans_matrix

def GeneratHead():

    choice_sublist_list = [[-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0]]
    random_index = random.randint(0,7)
    ans_list = random.sample(choice_sublist_list, 1)

    return choice_sublist_list[random_index]

def RandomNeighber(true_location_sublist, target_location):

    point_pool_list = []
    for i in [true_location_sublist[0] - 1, true_location_sublist[0], true_location_sublist[0] + 1]:
        for j in [true_location_sublist[1] - 1, true_location_sublist[1], true_location_sublist[1] + 1]:
            if i != true_location_sublist[0] and j != true_location_sublist[1]:
                if i != target_location[0] and j != target_location[1]:
                    point_pool_list.append([i,j])

    ans_list_list = random.sample(point_pool_list, 1)
    result_sublist = ans_list_list[0]

    return result_sublist

def CheckInBound(position_sublist):

    result = False
    if position_sublist[0] < 1 or position_sublist[0] > 15 or position_sublist[1] < 1 or position_sublist[1] > 10:
        result = True

    return result


def Move(true_location_sublist, head_sublist):

    target_location_sublist = [true_location_sublist[0] + head_sublist[0], true_location_sublist[1] + head_sublist[1]]

    if random.random() < 0.2:
        target_location_sublist = RandomNeighber(true_location_sublist, target_location_sublist)

    if CheckInBound(target_location_sublist):
        return [true_location_sublist, True]
    else:
        return [target_location_sublist, False]


Simulate()