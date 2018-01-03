'''
EECS498
JiachengZhu jiachzhu@umich,edu

'Bayes network CPT'

Need to be modified
'''
from numpy import *
import random

class BayesianNode:
    def __init__(self, name, table, type):
        self.name_str = name
        self.table = table
        self.type = type
        self.parent = []
        self.child = []
        self.table = table
        self.result = None



def GenerateNetwork(field_one, field_two, field_three, field_four):
    cloudy_table_dict = {'t':0.5, 'f':0.50}
    cloudy_node = BayesianNode('cloudy', cloudy_table_dict, field_one)

    sprinkler_table_dict = {'t':0.10, 'f':0.50}
    sprinkler_node = BayesianNode('sprinker', sprinkler_table_dict, field_two)

    rain_table_dict = {'t':0.80, 'f':0.20}
    rain_node = BayesianNode('rain', rain_table_dict, field_three)

    wetgrass_table_dict = {'tt':0.99, 'tf':0.90, 'ft':0.90, 'ff':0.01}
    wetgrass_node = BayesianNode('wetgrass', wetgrass_table_dict, field_four)

    cloudy_node.child = [sprinkler_node, rain_node]

    sprinkler_node.parent = [cloudy_node]
    sprinkler_node.child = [wetgrass_node]

    rain_node.parent = [cloudy_node]
    rain_node.child = [wetgrass_node]

    wetgrass_node.parent = [sprinkler_node, rain_node]

    network_dict = {'cloudy':cloudy_node, 'sprinkler':sprinkler_node, 'rain':rain_node, 'wetgrass':wetgrass_node}

    return network_dict

def PriorSample(network_dict):

    result_list = []
    cloud_table = network_dict['cloudy'].table
    cloud_result_char = ProbabilityTrueFlase(cloud_table['t'])
    result_list.append(cloud_result_char)

    sprinkler_table = network_dict['sprinkler'].table
    sprinkler_result_char = ProbabilityTrueFlase(sprinkler_table[cloud_result_char])
    result_list.append(sprinkler_result_char)

    rain_table = network_dict['rain'].table
    rain_result_char = ProbabilityTrueFlase(rain_table[cloud_result_char])
    result_list.append(rain_result_char)

    wetgrass_table = network_dict['wetgrass'].table
    wetgrass_result_char = ProbabilityTrueFlase(wetgrass_table[ sprinkler_result_char + rain_result_char])
    result_list.append(wetgrass_result_char)

    return result_list


def Consistent_x_e(x_list, e_list):

    consistent_result = False
    for i in range(len(x_list)):
        if e_list[i] == 't' or e_list[i] == 'f':
            if x_list[i] == e_list[i]:
                consistent_result = True
            else:
                return False

    return consistent_result




def ProbabilityTrueFlase(p):
    key = random.random()
    if key < p:
        ans = 't'
    else:
        ans = 'f'
    return ans

