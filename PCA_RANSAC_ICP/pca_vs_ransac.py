#!/usr/bin/env python
'''
EECS498
JiachengZhu jiachzhu@umich,edu

'PCA vs RANSAC'

Need to be modified
'''
import utils
import numpy
import time
import random
import matplotlib
###YOUR IMPORTS HERE###

from numpy import *
from RANSACmethod import *
import time
import matplotlib.pyplot as plt
###YOUR IMPORTS HERE###

def add_some_outliers(pc,num_outliers):
    pc = utils.add_outliers_centroid(pc, num_outliers, 0.75, 'uniform')
    random.shuffle(pc)
    return pc

def main():
    #Import the cloud
    pc = utils.load_pc('cloud_pca.csv')

    num_tests = 10
    fig = None
    error_list_pca = []
    error_list_ransac = []
    pc_pca_outliner_list_global = []
    pc_ransac_outliner_list_global = []
    pca_time_list = []
    ransac_time_list = []
    for i in range(0,num_tests):
        pc = add_some_outliers(pc,10) #adding 10 new outliers for each test
        fig = utils.view_pc([pc])

        ###YOUR CODE HERE###
        #Compute Mean Point
        pc2 = pc
        pc2_mat = utils.convert_pc_to_matrix(pc2)
        mean_mat = mean(pc2_mat, axis=1)

        '''
        Start PCA
        '''
        pca_start_time = time.clock()
        size = pc2_mat.shape
        minus_mat = zeros((size[0], size[1]))
        minus_mat[0, :] = mean_mat[0]
        minus_mat[1, :] = mean_mat[1]
        minus_mat[2, :] = mean_mat[2]
        pc2_pca_mat = pc2_mat - minus_mat
        q_mat = cov(pc2_pca_mat)
        u_mat, s_mat, v_mat_t = linalg.svd(q_mat)
        u_mat_mat = matrix(u_mat)

        normal_vctr_mat = u_mat_mat[:, 2]

        utils.draw_plane(fig, normal_vctr_mat, mean_mat, color=(0.0, 1, 0.0, 0.5), length=[-0.7, 0.7],
                         width=[-0.8, 0.8])

        A_para = float(normal_vctr_mat[0])
        B_para = float(normal_vctr_mat[1])
        C_para = float(normal_vctr_mat[2])
        D_para = -A_para*float(mean_mat[0]) - B_para*float(mean_mat[1]) - C_para*float(mean_mat[2])



        pca_para_list = [-A_para/C_para, -B_para/C_para, -D_para/C_para]

        pc_pca_inline_list_it = []
        pc_pca_out_list_it = []
        for mat in pc:
            if DistancePointPlane(mat, pca_para_list) < 0.1:
                # print 'found pca inliners'
                pc_pca_inline_list_it.append(mat)
            else:
                pc_pca_out_list_it.append(mat)

        error_pca = ErrorPointListPlane(pc_pca_inline_list_it, pca_para_list)

        error_list_pca.append(error_pca)
        pc_pca_outliner_list_global.append(len(pc_pca_out_list_it))

        pca_end_time = time.clock()
        pca_time = pca_end_time - pca_start_time

        pca_time_list.append(pca_time)

        '''
        Start Ransac
        '''

        ransac_start_time = time.clock()
        ans = RANSACsolve(pc, 500, 100, 0.15)

        [a, b, c] = ans[0]
        inliers_mat_list = ans[2]
        mean_point_ransac = matrix([[float(mean_mat[0])], [float(mean_mat[1])], [a * float(mean_mat[0]) + b * float(mean_mat[1]) + c]])
        normal_ransac = matrix([[a], [b], [-1]])

        utils.draw_plane(fig, normal_ransac, mean_point_ransac, color=(1.0, 0.0, 0.0, 0.5), length=[-0.7, 0.7],
                         width=[-0.8, 0.8])

        error_ransac = ErrorPointListPlane(inliers_mat_list, ans[0])
        error_list_ransac.append(error_ransac)
        #raw_input("Press enter for next test:")
        #time.sleep(5)
        matplotlib.pyplot.close(fig)

        pc_ransac_out_list_it = []
        for mat in pc:
            result = IfPointMatInList(mat, inliers_mat_list)
            if result[1] == False:
                pc_ransac_out_list_it.append(mat)
        pc_ransac_outliner_list_global.append(len(pc_ransac_out_list_it))

        ransac_end_time = time.clock()

        ransac_time_list.append(ransac_end_time - ransac_start_time)

        '''
        compute the inliners for PCA
        '''

        if i == (num_tests - 1):
            pc_pca_inline_list = []
            pc_pca_out_list = []
            for mat in pc:
                if DistancePointPlane(mat, pca_para_list) < 0.15:
                    #print 'found pca inliners'
                    pc_pca_inline_list.append(mat)
                else:
                    pc_pca_out_list.append(mat)
            #print 'pc_pca_inline_list = ', pc_pca_inline_list
            #print 'len(pc_pca_inline_list) =', len(pc_pca_inline_list)

            fig_pca = utils.view_pc([pc_pca_inline_list], color='r')
            fig_pca = utils.view_pc([pc_pca_out_list], fig_pca, color='b')
            fig_pca.set_label('pca')

            utils.draw_plane(fig_pca, normal_vctr_mat, mean_mat, color=(0.0, 1, 0.0, 0.5), length=[-0.7, 0.7],
                             width=[-0.8, 0.8])



            '''
            compute the outliners for RANSAC
            '''
            pc_ransac_inline_list = ans[2]
            pc_ransanc_out_list = []

            for mat in pc:
                result = IfPointMatInList(mat, pc_ransac_inline_list)
                if result[1] == False:
                    pc_ransanc_out_list.append(mat)

            fig_ransac = utils.view_pc([pc_ransac_inline_list], color='r')
            fig_ransac = utils.view_pc([pc_ransanc_out_list], fig_ransac, color='b')
            fig_ransac.set_label('ransac')
            utils.draw_plane(fig_ransac, normal_ransac, mean_point_ransac, color=(1.0, 0.0, 0.0, 0.5), length=[-0.7, 0.7],
                             width=[-0.8, 0.8])
            #fig_ransac.add_title('')

            print 'len(pc_pca_inline_list) =', len(pc_pca_inline_list)
            print 'len( pc_ransac_inline_list) =', len( pc_ransac_inline_list)



        ###YOUR CODE HERE###

    print 'error_list_pca =', error_list_pca
    print 'error_list_ransac =', error_list_ransac
    plt.figure(3)
    plt.title("Error vs outliners depends on Iteration Red-RANSAC Green-PCA")
    xvals = arange(0, 100, 10)
    yvals = array(error_list_pca)
    yvals_ransac = array(error_list_ransac)
    plt.plot(xvals, yvals, c='g')
    #plt.title("Error of PCA vs outliners")

    plt.plot(xvals, yvals_ransac, c='r')
    #plt.title("Error of RANSAC vs outliners")

    plt.figure(4)
    plt.title("Error vs outliners numbers Red-RANSAC Green-PCA")
    xval_srd_pca = array(pc_pca_outliner_list_global)
    xval_srd_ransac = array(pc_ransac_outliner_list_global)
    yval_srd_pca = array(error_list_pca)
    yval_srd_ransac = array(error_list_ransac)

    plt.scatter(xval_srd_pca, yval_srd_pca, c='g')
    plt.scatter(xval_srd_ransac, yval_srd_ransac, c='r')
    plt.plot()

    print 'ransac_time_list =', ransac_time_list
    print 'pca_time_list =', pca_time_list


    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
