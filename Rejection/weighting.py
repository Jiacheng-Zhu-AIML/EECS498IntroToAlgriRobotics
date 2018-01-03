'''
EECS498
JiachengZhu jiachzhu@umich,edu

'Weighting Rejection'

Need to be modified
'''

from BayesStructure import *
from numpy import *

f = open('inference.txt')

lines = f.readlines()
type_list= (lines[0][:-1]).split(',')
number_samples = int(lines[1])

network_dict = GenerateNetwork(type_list[0], type_list[1], type_list[2], type_list[3])

def WeightedSample(network_dict, type_list):

    result_list = []
    w = 1.0
    cloud_table = network_dict['cloudy'].table
    sprinkler_table = network_dict['sprinkler'].table
    rain_table = network_dict['rain'].table
    wetgrass_table = network_dict['wetgrass'].table

    if type_list[0] == 't' or type_list[0] == 'f' :
        cloud_result_char = type_list[0]
        result_list.append(cloud_result_char)
        w = w * cloud_table[cloud_result_char]
    else:
        cloud_result_char = ProbabilityTrueFlase(cloud_table['t'])
        result_list.append(cloud_result_char)

    if type_list[1] == 't' or type_list[1] == 'f' :
        sprinkler_result_char = type_list[1]
        result_list.append(sprinkler_result_char)

        if sprinkler_result_char == 't':
            w_sprinkler = sprinkler_table[cloud_result_char]
        elif sprinkler_result_char == 'f':
            w_sprinkler = 1 - sprinkler_table[cloud_result_char]
        w = w * w_sprinkler
    else:
        sprinkler_result_char = ProbabilityTrueFlase(sprinkler_table[cloud_result_char])
        result_list.append(sprinkler_result_char)

    if type_list[2] == 't' or type_list[2] == 'f' :
        rain_result_char = type_list[2]
        result_list.append(rain_result_char)

        if rain_result_char == 't':
            w_rain = rain_table[cloud_result_char]
        elif rain_result_char == 'f':
            w_rain = 1 - rain_table[cloud_result_char]
        w = w * w_rain
    else:
        rain_result_char = ProbabilityTrueFlase(rain_table[cloud_result_char])
        result_list.append(rain_result_char)

    if type_list[3] == 't' or type_list[3] == 'f' :
        wetgrass_result_char = type_list[3]
        result_list.append(wetgrass_result_char)

        if wetgrass_result_char == 't':
            w_wetgrass = wetgrass_table[sprinkler_result_char + rain_result_char]
        elif wetgrass_result_char == 'f' :
            w_wetgrass = 1 - wetgrass_table[sprinkler_result_char + rain_result_char]
        w = w * w_wetgrass
    else:
        wetgrass_result_char = ProbabilityTrueFlase(wetgrass_table[sprinkler_result_char + rain_result_char])
        result_list.append(wetgrass_result_char)

    return [result_list, w]


def LikelihoodeWeighting(type_list, network_dict, number):

    w_true_number = 0.0
    w_false_number = 0.0
    q_index = type_list.index('q')
    name_list = ['cloudy', 'sprinkler', 'rain', 'wetgrass']

    for i in range(number):
        weightsample_result = WeightedSample(network_dict, type_list)

        result_iterate_list= weightsample_result[0]
        result_w_number = weightsample_result[1]

        if result_iterate_list[q_index] == 't' :
            w_true_number = w_true_number + result_w_number
        elif result_iterate_list[q_index] == 'f' :
            w_false_number = w_false_number + result_w_number

    result = [w_true_number / (w_true_number + w_false_number), w_false_number / (w_true_number + w_false_number)]
    #print name_list[q_index], '=', result

    return result


#ans = LikelihoodeWeighting(type_list, network_dict, 10)
#print ans

