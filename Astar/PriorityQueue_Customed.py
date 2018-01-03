'''
EECS498 HW03 A-star
JiachengZhu jiachzhu@umich,edu

'A-Star Path Finding Algorithm '
Node class

'''

from Queue import PriorityQueue
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *

#defines a basic node class
class Node:
    def __init__(self, position, id_in, parentid_in,distance):
        self.x = position[0]
        self.y = position[1]
        self.theta = position[2]
        self.id = id_in
        self.parentid = parentid_in
        self.position = position
        self.distacne = distance
        self.detected = False
        self.parentnode = None



q = PriorityQueue()
q.put((1.46, Node([2,3,0.3],1,0,9)))
q.put((2.6, Node([5,2,0.1],2,1,9)))
q.put((5.6, Node([2,3,0.3],3,2,9)))
q.put((0.6, Node([4,3,0.2],4,1,9)))

print 'q = ',q

print 'q.get() = ', q.get()
queued_one = q.get()

print 'queued_one[1].position = ',queued_one[1].position
# print 'Node() =',Node(id = 2)