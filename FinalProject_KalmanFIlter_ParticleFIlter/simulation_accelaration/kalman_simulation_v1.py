import numpy as np
import matplotlib.pyplot as plt
import time
from time import sleep, ctime
# a=np.matrix([[0.],[0.],[1.]])
# b=np.array([[0.],[0.],[1.]])
# b[0][0]
# a[0,0]
# np.random.randn(3,1)*a
# a
class mobile_robot:
    def __init__(self,vector_in,t_in=1):
        self.t = t_in # time interval

        self.p = vector_in[0,0]
        self.v = vector_in[1,0]
        self.a = vector_in[2,0]
        self.vector = vector_in

        self.p_f = vector_in[0,0]
        self.v_f = vector_in[1,0]
        self.a_f = vector_in[2,0]
        self.vector_f = vector_in
        self.var = np.eye(3)*1000000 # pretty sure where it is at the beginning



    def renew_t(self,vector_new):
        self.p = vector_new[0,0]
        self.v = vector_new[1,0]
        self.a = vector_new[2,0]
        self.vector = vector_new

    def Move(self):
        p_new = self.p + self.v*self.t + 0.5*self.a*self.t**2
        v_new = self.v + self.a*self.t
        a_new = self.a
        vector_new = np.matrix([[p_new],[v_new],[a_new]]) + np.random.randn(3,1)
        self.renew_t(vector_new)
        return self.vector

    def Sensor(self):
        return self.vector+np.random.randn(3,1)*5

    def K_Filter(self):
        F = np.matrix([[1,self.t,0.5*self.t**2],[0,1,self.t],[0,0,1]])
        H = np.eye(3)
        # H = np.matrix([[1,0,0]])
        var_x = np.eye(3)
        var_z = np.eye(3)*25
        # var_z = 25*np.matrix([[1]])

        K_new = (F*self.var*F.T+var_x)*H.T*(H*(F*self.var*F.T+var_x)*H.T+var_z).I
        # print F,K_new,self.vector_f,self.Sensor()
        vector_new = F*self.vector_f + K_new*(self.Sensor()-H*F*self.vector_f)
        # vector_new = F*self.vector_f + K_new*(self.Sensor()[0]-H*F*self.vector_f)
        var_new = (np.eye(3)-K_new*H)*(F*self.var*F.T+var_x)

        self.renew_f(vector_new,var_new)

        return vector_new #,var_new

    def K_Filter_1(self):
        F = np.matrix([[1,self.t,0.5*self.t**2],[0,1,self.t],[0,0,1]])
        # H = np.eye(3)
        H = np.matrix([[1,0,0]])
        var_x = np.eye(3)
        # var_z = np.eye(3)*25
        var_z = 25*np.matrix([[1]])

        K_new = (F*self.var*F.T+var_x)*H.T*(H*(F*self.var*F.T+var_x)*H.T+var_z).I
        # print F,K_new,self.vector_f,self.Sensor()
        # vector_new = F*self.vector_f + K_new*(self.Sensor()-H*F*self.vector_f)
        vector_new = F*self.vector_f + K_new*(self.Sensor()[0]-H*F*self.vector_f)
        var_new = (np.eye(3)-K_new*H)*(F*self.var*F.T+var_x)

        self.renew_f(vector_new,var_new)

        return vector_new #,var_new

    def renew_f(self,vector_new_f,var_new):
        self.p_f = vector_new_f[0,0]
        self.v_f = vector_new_f[1,0]
        self.a_f = vector_new_f[2,0]
        self.vector_f = vector_new_f
        self.var = var_new


def main():
    start = np.matrix([[0.],[0.],[3.]])
    pr2 = mobile_robot(start)
    l1 = 40
    t = 1 # time interval
    true_locations = [start[0,0]]
    sensed_locations = [start[0,0]]
    filtered_locations = [start[0,0]]
    filtered_locations_1 = [start[0,0]]
    for t1 in range(l1):
        pr2.Move()
        if not t1%t:
            # print pr2.vector
            # print pr2.Sensor()
            true_locations.append(pr2.vector[0,0])
            sensed_locations.append(pr2.Sensor()[0,0])
            # filtered_locations.append(pr2.K_Filter()[0,0])
            filtered_locations_1.append(pr2.K_Filter_1()[0,0])

    print '==================true_locations'
    print true_locations
    print '==================sensed_locations'
    print sensed_locations
    print '==================filtered_locations'
    print filtered_locations
    print len(true_locations)
    plt.figure(1)
    plt.plot(range(len(true_locations)),true_locations,'ko',label = 'true position')
    plt.plot(range(len(true_locations)),sensed_locations,'r.',label = 'sensed position')
    plt.plot(range(len(true_locations)),filtered_locations_1,'g*',label = 'filtered location')
    plt.xlabel('time')
    plt.ylabel('position')
    # plt.legend(('filtered position','sensored error'),loc='upper right')
    plt.title('Kalman Filter with 1 State Varibles')
# plt.plot(range(len(true_locations)),filtered_locations_1,'b^')
    # plt.show()
    ####### Ahhhhhhhhhh HOW to do this ???????
    # plt.figure(1)
    #
    # circle1 = plt.Circle((target_location[0],target_location[1]), criterion, color='r')
    # plt.gcf().gca().add_artist(circle1)
    #
    # true_locations_x = []
    # true_locations_y = []
    # for i in range(0,len(true_locations)):
    #     true_locations_x.append(true_locations[i][0])
    #     true_locations_y.append(true_locations[i][1])
    # plt.plot(true_locations_x,true_locations_y,'k.-',alpha=0.9)
    #
    # sensed_locations_x = []
    # sensed_locations_y = []
    # for i in range(0,len(sensed_locations)):
    #     sensed_locations_x.append(sensed_locations[i][0])
    #     sensed_locations_y.append(sensed_locations[i][1])
    # plt.plot(sensed_locations_x,sensed_locations_y,'ro',alpha=0.5)
    #
    # filtered_locations_x = []
    # filtered_locations_y = []
    # for i in range(0,len(filtered_locations)):
    #     filtered_locations_x.append(filtered_locations[i][0])
    #     filtered_locations_y.append(filtered_locations[i][1])
    # plt.plot(filtered_locations_x,filtered_locations_y,'bx-',alpha=0.5)
    #
    true_locations_x_array = np.array(true_locations)
    # true_locations_y_array = np.array(true_locations_y)
    sensed_locations_x_array = np.array(sensed_locations)
    # sensed_locations_y_array = np.array(sensed_locations_y)
    # filtered_locations_x_array = np.array(filtered_locations)
    filtered_locations_x_1_array = np.array(filtered_locations_1)
    # filtered_locations_y_array = np.array(filtered_locations_y)
    # distance_f = np.sqrt((true_locations_x_array-filtered_locations_x_array)**2)
    distance_f_1 = np.sqrt((true_locations_x_array-filtered_locations_x_1_array)**2)
    distance_s = np.sqrt((true_locations_x_array-sensed_locations_x_array)**2)

    # print 'average of filter distance error:',sum(distance_f)/len(distance_f)
    print 'average of filter_1 distance error:',sum(distance_f_1)/len(distance_f_1)
    print 'average of sensor distance error',sum(distance_s)/len(distance_s)

    plt.figure(2)
    # plt.plot(range(0,distance_f.shape[0]),list(distance_f),'r.-',range(0,distance_s.shape[0]),list(distance_s),'g.-')
    plt.plot(range(0,distance_s.shape[0]),list(distance_s),'g.-',range(0,distance_f_1.shape[0]),list(distance_f_1),'b.-')
    plt.xlabel('iterations')
    plt.ylabel('error(distance)')
    # plt.legend(('filter distance error','sensor distance error'),loc='upper right')
    plt.legend(('filter distance error','filter_1 distance error'),loc='upper right')
    plt.title('Kalman Filter with 1 State Varibles')
    plt.show()
    # distance = np.sqrt((true_locations[i][0]-target_location[0])**2+(true_locations[i][1]-target_location[1])**2)
    # return distance_f,distance_s,criterion

if __name__ == '__main__':
    main()
