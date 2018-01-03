from BayesStructure import *
from numpy import *

f = open('inference.txt')

lines = f.readlines()
type_list= (lines[0][:-1]).split(',')
number_samples = int(lines[1])

network_dict = GenerateNetwork(type_list[0], type_list[1], type_list[2], type_list[3])

#print 'ProbabilityTrueFlase(0.9) =', ProbabilityTrueFlase(0.9)

# result_test = PriorSample(network_dict)
# print 'result_test=', result_test
# print 'type_list =', type_list
# print type_list.index('q')
#
# ans = Consistent_x_e(result_test, type_list)
# print 'ans =', ans

def RejectionSampling(type_list, network_dict, number):

    consist_number = 0
    q_true_number = 0.0
    q_false_number = 0.0
    q_index = type_list.index('q')
    name_list = ['cloudy', 'sprinkler', 'rain', 'wetgrass']
    finding = 1
    for i in range(number):
    #while consist_number < number:
        result_list = PriorSample(network_dict)

        if Consistent_x_e(result_list, type_list):
            consist_number = consist_number + 1

            if result_list[q_index] == 't':
                q_true_number = float(q_true_number) + 1.0
            elif result_list[q_index] == 'f':
                q_false_number = float(q_false_number) + 1.0

        #print 'i =', i
    #print consist_number, 'samples consists'
    if q_true_number == 0:
        result = [0, 1]
    else:
        result = [q_true_number/(q_true_number + q_false_number), q_false_number/(q_true_number + q_false_number)]
    #print name_list[q_index], '=' , result
    return result

#test_list_two = ['q', '-', 't', 'f']
ans = RejectionSampling(type_list, network_dict, 10)
print ans
