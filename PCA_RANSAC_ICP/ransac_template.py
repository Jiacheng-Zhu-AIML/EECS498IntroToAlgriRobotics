#!/usr/bin/env python
'''
EECS498
JiachengZhu jiachzhu@umich,edu

'RANSAC'

Need to be modified
'''
import utils
import numpy
###YOUR IMPORTS HERE###
from RANSACmethod import *
from utils import *
###YOUR IMPORTS HERE###


def main():
    #Import the cloud
    pc = utils.load_pc('cloud_ransac.csv')
    ###YOUR CODE HERE###
    pc2 = pc[0:10]
    pc3 = pc[20:60]

    # print pc2
    # print len(pc2)
    # print type(pc2)
    #
    # for one in pc2:
    #     print one
    #     print type(one)
    #
    # test = (pc2[0].T).tolist()
    # print 'pc2[0][0] =', float(pc2[0][0])
    # print 'type(pc2[0][0]) =', type(pc2[0][0])
    #
    # print 'test =',test
    #
    # print type(test)
    #
    # lst_sq_result = LeastSquare(pc2)
    # print 'LeastSquare(pc2) =', lst_sq_result
    # Show the input point cloud
    #utils.view_pc([pc])

    #PlotThreeDPointClouds(pc2, pc3)
    #Fit a plane to the data using ransac

    #judge_test = IfPointMatInList(pc[10],pc)

    #print 'judge_test =', judge_test


    ans = RANSACsolve(pc, 1000, 100, 0.15)
    [a, b, c] = ans[0]
    inliers_mat_list = ans[2]

    #print 'inliers_mat_list =', inliers_mat_list
    #print 'len(inliers_mat_list) =', len(inliers_mat_list)
    print '( z = ax+by+c ) a,b,c = ', ans[0]

    pc2_mat = utils.convert_pc_to_matrix(pc)
    mean_mat = mean(pc2_mat, axis=1)

    out_pc_list = []

    for mat_one in pc:
        result = IfPointMatInList(mat_one, inliers_mat_list)
        if result[1] == False:
            out_pc_list.append(mat_one)

#    print 'len(out_pc_list) =', len(out_pc_list)

    mean_point = matrix([[float(mean_mat[0])], [float(mean_mat[1])], [a*float(mean_mat[0]) + b*float(mean_mat[1]) + c]])
    normal = matrix([[a], [b], [-1]])

    #Show the resulting point cloud
    fig = view_pc([ans[2]], color='r')
    fig = view_pc([out_pc_list],fig, color='b')

    #Draw the fitted plane
    fig = draw_plane(fig, normal, mean_point, color=(0.0, 1, 0.0, 0.5), length=[-0.7, 0.7], width=[-0.8, 0.8])

    ###YOUR CODE HERE###
    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
