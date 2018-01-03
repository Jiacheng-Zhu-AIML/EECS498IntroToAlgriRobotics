'''
EECS498 HW03 RRT
JiachengZhu jiachzhu@umich,edu

'RRT Tree'
Class Bracnh
Some times there will be openrave error, just try again please. The algorithm does work for most of the times

'''

import openravepy
#from RRT_Path_Finding import DetectConifgCollsion
if not __openravepy_build_doc__:
    from openravepy import *


#Defines a tree class
class Branch:
    def __init__(self,vector,branch_id,parent_branch_id,target_config):
        self.config_a = vector[0]
        self.config_b = vector[1]
        self.branchid = branch_id
        self.parentid = parent_branch_id
        self.parentbranch = None
        self.length = 0
        self.targetconfig = target_config
        self.tested = False

