#!/usr/bin/env python
'''
EECS498
JiachengZhu jiachzhu@umich,edu

'PCA'

Need to be modified
'''
import utils
import numpy
###YOUR IMPORTS HERE###
from numpy import *
###YOUR IMPORTS HERE###


def main():

    #Import the cloud
    pc = utils.load_pc('cloud_pca.csv')

    pc2 = pc

    ###YOUR CODE HERE###
    # Show the input point cloud
    fig = utils.view_pc([pc])

    # Rotate the points to align with the XY plane

    pc2_mat = utils.convert_pc_to_matrix(pc2)
    mean_mat = mean(pc2_mat, axis=1)
    size = pc2_mat.shape
    minus_mat = zeros((size[0], size[1]))
    minus_mat[0,:] = mean_mat[0]
    minus_mat[1,:] = mean_mat[1]
    minus_mat[2,:] = mean_mat[2]
    pc2_mat = pc2_mat - minus_mat
    q_mat = cov(pc2_mat)
    u_mat, s_mat, v_mat_t = linalg.svd(q_mat)

    v_mat = v_mat_t.T
    new_pc2_mat = v_mat*pc2_mat
    new_pc2_pc = utils.convert_matrix_to_pc(new_pc2_mat)
    print '(w) V_mat =', v_mat.T

    # Show the resulting point cloud

    fig = utils.view_pc([new_pc2_pc], fig, color='r')

    # Rotate the points to align with the XY plane AND eliminate the noise

    v_elim_mat = diag(s_mat)*diag(s_mat)

    for i in range(0, len(v_elim_mat)):
        if v_elim_mat[i][i] < 0.0001:
            elim_index = i
    v_mat_elim = v_mat
    v_mat_elim[elim_index,:] = 0
    print '(W)v_mat = ', v_mat_elim.T
    new_pc2_mat_elim = v_mat_elim*pc2_mat
    new_pc2_pc_elim = utils.convert_matrix_to_pc(new_pc2_mat_elim)

    # Show the resulting point cloud

    fig = utils.view_pc([new_pc2_pc_elim], fig, color='y')

    #fit a plane to the cloud

    u_mat_mat = matrix(u_mat)

    normal_vctr_mat = u_mat_mat[:, 2]

    utils.draw_plane(fig, normal_vctr_mat, mean_mat,  color=(0.0, 1, 0.0, 0.5), length=[-0.7, 0.7], width=[-0.8, 0.8])

    ###YOUR CODE HERE###


    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
