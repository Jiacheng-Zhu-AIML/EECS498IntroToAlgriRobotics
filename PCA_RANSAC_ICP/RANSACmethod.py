'''
EECS498
JiachengZhu jiachzhu@umich,edu

'RANSAC functions'

Need to be modified
'''
from numpy import *
from scipy.optimize import leastsq
import matplotlib.pyplot as plt
import pylab as pl
import random
from mpl_toolkits.mplot3d import Axes3D

def RANSACsolve(point_mat_list, k_iterate, n, threshold):
    '''
    :param point_mat_list: Matrixlist [matrix([[x1],[y1],[z1]]),matrix([[x1],[y1],[z1]])]
    :param k_iterate: # of iterations,
    :param n: minimum number of consensus points required
    :param threshold: threshold for inliers,
    :return: [a, b, c] where plane is  z = a*x + b*y + c
    '''
    distance = 10000
    best_model_para = []
    model_list = []
    three_point = []
    for i in range(0, k_iterate):
        print i
        temp_pc_mat_list = point_mat_list[:]
        rdm_sbst_list = random.sample(point_mat_list, 3)
        first_fit_para = LeastSquare(rdm_sbst_list)
        consensus_set_list = []
        new_model_list = []
        for mat in temp_pc_mat_list:
            ###
            #if mat not in rdm_sbst_list:
            if 1 == True:
                 ###
                error_p_mat_plane = DistancePointPlane(mat, first_fit_para)
                if error_p_mat_plane < threshold:
                    consensus_set_list.append(mat)

        if len(consensus_set_list) > n:
            new_model_list = consensus_set_list + rdm_sbst_list
            second_fit_para = LeastSquare(consensus_set_list)


            # tt_error = 0
            # for point_mat in new_model_list:
            #     tt_error = tt_error + DistancePointPlane(point_mat, second_fit_para)**2
            # err_css_set_mod = tt_error/len(consensus_set_list)

            error = ErrorPointListPlane(consensus_set_list, second_fit_para)/len(consensus_set_list)


            if error < distance:
                print 'Now the smallest distance is'
                print distance
                model_list = new_model_list
                three_point = rdm_sbst_list
                distance = error
                best_model_para = second_fit_para
            print 'Smallest error is', distance,'Best model N is',len(new_model_list)

    return [best_model_para, three_point, model_list]


def LeastSquare(point_mat_list):
    '''
    :param point_mat_list: Matrixlist [matrix([[x1],[y1],[z1]]),matrix([[x1],[y1],[z1]])]
    :return: [a, b, c] where plane is  z = a*x + b*y + c
    '''

    def func(x, y, p):
        a, b, c = p
        return a * x + b * y + c

    def residuals(p, z, x, y):
        upper = abs(p[0]*x + p[1]*y - z + p[2])
        down = (p[0]**2 + p[1]**2 + 1) ** 0.5
        d = upper / down
        return d

    x_lst = []
    y_lst = []
    z_lst = []
    for mat_one in point_mat_list:
        x_lst.append(float(mat_one[0]))
        y_lst.append(float(mat_one[1]))
        z_lst.append(float(mat_one[2]))

    x_ary = array(x_lst)
    y_ary = array(y_lst)
    z_ary = array(z_lst)

    # print 'x_ary =', x_ary
    # print 'type(x_ary) =', type(x_ary)
    # print 'y_ary =', y_ary
    # print 'z_ary =', z_ary
    result_leastsq = leastsq(residuals, [0, 0, 0], args=(z_ary, x_ary, y_ary))
    para = result_leastsq[0]
    return para

def Planefunction(x, y, p):
    '''
    z = a*x + b*y + c
    :param x: x of the point
    :param y: y of the point
    :param p: parameters p =[a, b, c]
    :return: z = a*x + b*y + c
    '''
    a, b, c =p

    return a*x + b*y + c


def residuals(p, z, x, y):
    '''
    :param p: parameters p =[a, b, c]
    :param z: z = a*x + b*y + c
    :param x: x of the point
    :param y: y of the point
    :return: distance between point and plane z = a*x + b*y + c
    '''
    upper = abs(p[0]*x + p[1]*y - z + p[2])
    down = (p[0]**2 + p[1]**2 + 1)**0.5
    d = upper/down

    return d

def DistancePointPlane(matrix_of_point, p):
    x = float(matrix_of_point[0])
    y = float(matrix_of_point[1])
    z = float(matrix_of_point[2])
    upper = abs(p[0]*x + p[1]*y - z + p[2])
    down = (p[0]**2 + p[1]**2 + 1)**0.5
    d = upper/down

    return d

def ErrorPointListPlane(pc_list, plane_para):
    tt_error = 0
    for point_mat in pc_list:
        tt_error = tt_error + DistancePointPlane(point_mat, plane_para) ** 2
    error = tt_error

    return error


def PlotThreeDPointClouds(pc1_mat_lst, pc2_mat_lst):
    '''
    :param pc1_mat_lst: Matrixlist [matrix([[x1],[y1],[z1]]),matrix([[x1],[y1],[z1]])]
    :param pc2_mat_lst: Matrixlist [matrix([[x1],[y1],[z1]]),matrix([[x1],[y1],[z1]])]
    :return: [x1_ary, y1_ary, z1_ary]
    '''
    ax = plt.figure().add_subplot(111, projection='3d')
    pc_one_arys = PointcloudMatrixListToXYZArray(pc1_mat_lst)
    pc_two_arys = PointcloudMatrixListToXYZArray(pc2_mat_lst)

    ax.scatter(pc_one_arys[0], pc_one_arys[1], pc_one_arys[2])
    ax.scatter(pc_two_arys[0], pc_two_arys[1], pc_two_arys[2])

    pl.show()


def PointcloudMatrixListToXYZArray(pc_mat_list):
    x_lst = []
    y_lst = []
    z_lst = []
    for mat_one in pc_mat_list:
        x_lst.append(float(mat_one[0]))
        y_lst.append(float(mat_one[1]))
        z_lst.append(float(mat_one[2]))

    x1_ary = array(x_lst)
    y1_ary = array(y_lst)
    z1_ary = array(z_lst)
    return [x1_ary, y1_ary, z1_ary]

def GenearatePointCloudListFromPara(p, delta = 50,x_init = -0.5, x_end = 1.5, y_init = -0.5, y_end = 1.5):
    result = []
    a = p[0]
    b = p[1]
    c = p[2]
    for ix in range(0, delta):
        for iy in range(0, delta):
            x = (x_end - x_init)*ix/float(delta) + x_init
            y = (y_end - y_init)*iy/float(delta) + y_init
            z = x*a + y*b + c
            result.append(matrix([[x], [y], [z]]))

    return result

def IfPointMatInList(pc_point_mat, pc_list):
    judge = False
    i = -1
    for one in pc_list:
        i = i + 1
        if pc_point_mat[0][0] == one[0][0] and pc_point_mat[1][0] == one[1][0] and pc_point_mat[2][0] == one[2][0]:
            judge = True
            break
    # if judge == None:
    #     i = -1
    return [i,judge]
