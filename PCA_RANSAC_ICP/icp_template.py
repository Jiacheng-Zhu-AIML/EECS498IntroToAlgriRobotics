#!/usr/bin/env python
'''
EECS498
JiachengZhu jiachzhu@umich,edu

'ICP'

Need to be modified
'''
import utils
import numpy
import matplotlib.pyplot as plt
###YOUR IMPORTS HERE###
from ICPmethod import *
import matplotlib.pyplot as plot
###YOUR IMPORTS HERE###


def main():
    #Import the cloud
    pc_source = utils.load_pc('cloud_icp_source.csv')


    ###YOUR CODE HERE###
    pc_target = utils.load_pc('cloud_icp_target2.csv') # Change this to load in a different target
    icp_result = ICPfunction(pc_source, pc_target, 0.2)
    result_pc = icp_result[2]

    #print 'pc_source =', pc_source
    #print 'pc_target =', pc_target
    #print 'result_pc =', result_pc

    #print 'r_mat_list = ', icp_result[0]
    #print 't_mat_list = ', icp_result[1]

    r_final_mat = matrix([[1,0,0],[0,1,0],[0,0,1]])

    for one in icp_result[0]:
        r_final_mat = one*r_final_mat

    #t_final_mat = matrix([[0],[0],[0]])
    i = 0
    t_final_mat = icp_result[1][0]
    for i in range(1, len(icp_result[1])):
        t_final_mat = icp_result[0][i]*t_final_mat + icp_result[1][i]

    print 'r_final_mat = ', r_final_mat
    print 't_final_mat = ', t_final_mat

    pc_verify = []

    # for one in pc_source:
    #     temp = r_final_mat*one + t_final_mat
    #     pc_verify.append(temp)


    utils.view_pc([pc_source, pc_target, result_pc, pc_verify], None, ['b', 'r', 'g', 'y'], ['o', '^', 'o', 'o'])
    #plt.axis([-0.15, 0.15, -0.15, 0.15]) #origin
    plt.axis([-0.1, 0.1, -0.1, 0.1])  # target0
    #plt.axis([-0.15, 0.15, -0.15, 0.15])  # origin
    #plt.axis([-0.15, 0.15, -0.15, 0.15])  # origin
    #plt.axis([-0.15, 0.15, -0.15, 0.15])  # origin

    x_array = array(range(1, len(icp_result[3])+1))
    y_error_ary = array(icp_result[3])
    fig = plot.figure()
    plot.plot(x_array, y_error_ary)
    plot.xlabel('Iteration')
    plot.ylabel('Error')
    plot.scatter(x_array, y_error_ary)
    plot.title('Error vs iterations')


    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
