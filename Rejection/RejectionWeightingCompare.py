'''
EECS498
JiachengZhu jiachzhu@umich,edu

'Comparing with rejection and weighting'

Need to be modified
'''

from rejection import *
from weighting import *
from matplotlib import pyplot as plt
from numpy import *

type_list_one = ['f', '-', '-', 'q']
type_list_two = ['q', '-', 't', 'f']
type_list_three = ['t', '-', 'q', 't']

type_list = type_list_two

network_dict = GenerateNetwork(type_list[0], type_list[1], type_list[2], type_list[3])

rejection_result_list = []
weighting_result_list = []

for i in range(10, 1010, 20):

    number_samples = i
    rejection_result_value = 0
    weighting_result_value = 0
    print 'i =', i
    for j in range(30):
        rejection_result = RejectionSampling(type_list, network_dict, i)
        weighting_result = LikelihoodeWeighting(type_list, network_dict, i)

        rejection_result_value = rejection_result_value + rejection_result[0]
        weighting_result_value = weighting_result_value + weighting_result[0]

    rejection_result_list.append(rejection_result_value/30.0)
    weighting_result_list.append(weighting_result_value/30.0)

plt.plot(range(10, 1010, 20), rejection_result_list,'r')

plt.plot(range(10, 1010, 20), weighting_result_list, 'b')

plt.text(0, 0.6, 'red is rejection, blue is weighting')
plt.show()

