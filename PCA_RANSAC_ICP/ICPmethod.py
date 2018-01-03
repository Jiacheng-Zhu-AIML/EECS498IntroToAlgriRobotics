'''
EECS498
JiachengZhu jiachzhu@umich,edu

'ICP'

Need to be modified
'''

from numpy import *

def ICPfunction(pc_current_list, pc_target_list, e=0.1):

    r_mat_result = matrix([[1,0,0],
                           [0,1,0],
                           [0,0,1]])
    t_mat_result = matrix([[0],
                           [0],
                           [0]])
    r_mat_list = []
    t_mat_list = []

    find = 1
    error_last = 100
    c_pc_p_list = pc_current_list
    error_list = []

    while find > 0:

        c_pc_q_list = []

        for one in c_pc_p_list:
            pt_q_nst = FindNearestPoint(one, pc_target_list)
            c_pc_q_list.append(pt_q_nst)

        if find == -1:
            #####
            '''
            move the points according to centers
            '''
            source_center_ba = GetCentroids(pc_current_list)
            target_center_ba = GetCentroids(pc_target_list)

            r_first_mat = matrix([[1,0,0],
                                  [0,1,0],
                                  [0,0,1]])
            t_first_mat = target_center_ba - r_first_mat * source_center_ba
            result = [r_first_mat, t_first_mat]
            #c_pc_q_list = pc_target_list

            #####
        else:
            result = GetTransForm(c_pc_p_list, c_pc_q_list)

        r_mat = result[0]
        t_mat = result[1]
        r_mat_list.append(r_mat)
        t_mat_list.append(t_mat)

        error = ErrorPointClouds(c_pc_p_list, c_pc_q_list, r_mat, t_mat)
        print 'Now error is =', error
        error_list.append(error)

        if error < e or abs(error_last - error) < (e/100) or find == 100:
            r_mat_result = r_mat
            t_mat_result = t_mat

            print 'Found ! or end'
            find = -1
        error_last = error
        #update p
        new_p_list = []
        for one in c_pc_p_list:
            new_pt_mat = r_mat*one + t_mat
            new_p_list.append(new_pt_mat)
        c_pc_p_list = new_p_list
        find = find + 1
        print find

    return [r_mat_list, t_mat_list, c_pc_p_list, error_list]

def FindNearestPoint(point, point_list):

    result_mat = point_list[0]
    dist = 10000
    for one in point_list:
        dist_temp = linalg.norm(one - point)
        if dist_temp < dist:
            dist = dist_temp
            result_mat = one
    pt_nearest = result_mat
    return pt_nearest


def GetTransForm(pc_p_list, pc_q_list):
    p_ba_mat = GetCentroids(pc_p_list)
    q_ba_mat = GetCentroids(pc_q_list)

    '''
    Compute X_mat, Y_mat
    '''
    ####
    X_mat = matrix(zeros((3, len(pc_p_list))))
    Y_mat = matrix(zeros((3, len(pc_q_list))))
    index = 0
    for one in pc_p_list:
        deta_mat = one - p_ba_mat
        X_mat[:,index] = deta_mat
        index = index + 1
    index = 0
    for one in pc_q_list:
        Y_mat[:,index] = (one - q_ba_mat)
        index = index + 1
    ####
    S_mat = X_mat*(Y_mat.T)
    u_mat, sigma_mat, v_mat_t = linalg.svd(S_mat)
    vu_t_mat = (v_mat_t.T)*(u_mat.T)
    det_value = linalg.det(vu_t_mat)
    mid_mat = matrix([[1,0,0],
                      [0,1,0],
                      [0,0,det_value]])
    R_mat = v_mat_t.T * mid_mat * u_mat.T
    T_mat = q_ba_mat - R_mat*p_ba_mat
    return [R_mat, T_mat]


def GetCentroids(pc_mat_list):
    length = len(pc_mat_list)
    sum0 = 0
    sum1 = 0
    sum2 = 0
    for one in pc_mat_list:
        sum0 = sum0 + float(one[0][0])
        sum1 = sum1 + float(one[1][0])
        sum2 = sum2 + float(one[2][0])
    ans = matrix([[sum0/length],
                  [sum1/length],
                  [sum2/length]])
    return ans

def ErrorPointClouds(pc_p_list, pc_q_list, r_mat, t_mat):
    sum = 0
    index = 0
    for one in pc_p_list:
        nearesr_pt = FindNearestPoint(one, pc_q_list)
        temp = linalg.norm(r_mat*one + t_mat - nearesr_pt)
        sum = sum + temp
        index = index + 1
    return sum