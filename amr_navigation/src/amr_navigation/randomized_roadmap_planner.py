#!/usr/bin/env python
import rospy
import numpy as np

#=============================== YOUR CODE HERE ===============================
# Instructions: complete the currently empty RandomizedRoadmapPlanner class.
#               An instance of this class will be created by the path_planner
#               node. It should maintain a graph of points and the connections
#               between them.
#               The 'plan()' function should find a path between the requested
#               points, inserting new nodes and edges if necessary. Make sure
#               that it stops at some point in time if no path between the
#               points exists.
#
# Remark: it will be necessary to test points and line segments for emptiness.
#         The class is (as usual) ROS-independent, so the actual mechanism of
#         performing these tests is abstracted by two callback functions, which
#         the object receives during the construction. In order to test whether
#         e.g. the point (1, 4) is free you should do:
#
#             free = self.point_free_cb((1, 4))
#
# Hint: use the standard function 'math.uniform()' to generate the coordinates
#       for random points.
#
# Hint: if you decided to use 'pygraph' library for graph and search
#       implementations, make sure that the graph object is stored in a member
#       field called 'graph'. If this is the case, the nodes and edges of the
#       graph will be automatically visualized by the path_planner node after
#       each planning request.

import random
import rospy
import math

from pygraph.classes.graph import graph
from pygraph.classes.exceptions import NodeUnreachable
from pygraph.algorithms.heuristics.euclidean import euclidean
from pygraph.algorithms.minmax import heuristic_search

class RandomizedRoadmapPlanner:

    def __init__(self, point_free_cb, line_free_cb, dimensions):
        """
        Construct a randomized roadmap planner.

        'point_free_cb' is a function that accepts a point (two-tuple) and
        outputs a boolean value indicating whether the point is in free space.

        'line_free_cb' is a function that accepts two points (the start and the
        end of a line segment) and outputs a boolen value indicating whether
        the line segment is free from obstacles.

        'dimensions' is a tuple of tuples that define the x and y dimensions of
        the world, e.g. ((-8, 8), (-8, 8)). It should be used when generating
        random points.
        """
        rospy.loginfo('Initializing RandomizedRoadmapPlanner')

        self.point_free_cb = point_free_cb
        self.line_free_cb = line_free_cb
        self.dimensions = dimensions
        self.node_list = []
        self.graph = graph()
        self.heuristic = euclidean()
        self.node_ids = []


    #This function used to generate random node
    def generateRandomNode(self):
        a = self.dimensions[0]
        x = random.uniform(a[0],a[1])
        y = random.uniform(a[0],a[1])
        return (x,y)

    #This function used to add the randomly generated node to the node list
    def addNode(self):
        node = self.generateRandomNode()
        #Checking the new node is point free and is not near to any other existing nodes
        if self.point_free_cb(node) and self.isNearestNode(node):
            #Adding new node to the graph and local node list
            self.graph.add_node(self.generate_id(), attrs=[('position', node)])
            self.node_list.append(node)
            rospy.loginfo('New node added to graph')
            
    #This function is used to calculate the euclidean distance between the two nodes
    def calculateEuclidean(self,node_one,node_two):
        return math.sqrt((node_one[0] - node_two[0]) * (node_one[0] - node_two[0]) + (node_one[1] - node_two[1]) * (node_one[1] - node_two[1]))

    #This function is used to check whether the newly generated node is near to any other existing node
    def isNearestNode(self,node):
        euclidean_distances = []
        for val in self.node_list:
            euclidean_distances.append(abs(self.calculateEuclidean(node,val)))
        if min(euclidean_distances) > 1:
            return True
        return False

    #This function is used to generate random id for node
    def generate_id(self):
        id = 0
        while (id == 0):
            id = random.randint(1,10000)
            if id in self.node_ids:
                id = 0
        self.node_ids.append(id)
        return id

    #This function is used the get the node position for the given node id
    def get_node(self,node_id):
        for nid, attr in self.graph.node_attr.iteritems():
            if(nid == node_id):
                return attr[0][1]
                
    #This function is used to plan the shortest path between the start and end node
    def plan(self, point1, point2):

        a = self.dimensions[0]
        rospy.logwarn(np.random.uniform(a[0],a[1],30))
        rospy.logwarn(np.random.uniform(a[0],a[1],30))

        
        self.node_list = []
        result_node_id =[]
        result_path =[]

        #Adding start node to the graph and local node list
        start_node_id = self.generate_id()
        self.graph.add_node(start_node_id, attrs=[('position', point1)])
        self.node_list.append(point1)
        
        #Adding goal node to the graph and local node list
        goal_node_id = self.generate_id()
        self.graph.add_node(goal_node_id, attrs=[('position', point2)])
        self.node_list.append(point2)

        #Checking is there is any direct path between the start and goal node
        if self.line_free_cb(point1,point2):
            rospy.loginfo('Direct path found between start and goal node')
            return self.node_list

        #Generating 30 random nodes and adding it to graph
        while (len(self.node_list) < 30):
            self.addNode()
        
        rospy.loginfo('Creating possible edges process started...')
        #Generating possible edges between the generated nodes
        for nid, attr in self.graph.node_attr.iteritems():
            temp_id = nid
            temp_position = attr[0][1]
            for nid_1, attr_1 in self.graph.node_attr.iteritems():
                if(nid_1 != temp_id):
                    #Checking the line free between two nodes
                    if self.line_free_cb(temp_position,attr_1[0][1]):
                        if not self.graph.has_edge((temp_id,nid_1)):
                            #Adding the edge between two nodes with euclidean distance
                            self.graph.add_edge((temp_id,nid_1),self.calculateEuclidean(temp_position,attr_1[0][1]))
        rospy.loginfo('Creating possible edges process finished...') 

        #Optimizing the graph
        self.heuristic.optimize(self.graph)
        rospy.loginfo('Optimizing graph finished...') 
 
        try:
            #Performing A* search to find the shortest path between the start node and goal node
            rospy.loginfo('Finding shorted path using A* algorithm started') 
            result_node_id = heuristic_search(self.graph,start_node_id, goal_node_id, self.heuristic)
            rospy.loginfo('Shorteset path found...')
        except:  
            rospy.loginfo('No Shorteset path found...')
 
        #Creating the list for the shortest path between the start and goal node
        for name in result_node_id:
            result_path.append(self.get_node(name))
 
        print result_path
        rospy.loginfo('Planning completed...')
        return result_path


        """
        Plan a path which connects the two given 2D points.

        The points are represented by tuples of two numbers (x, y).

        Return a list of tuples where each tuple represents a point in the
        planned path, the first point is the start point, and the last point is
        the end point. If the planning algorithm failed the returned list
        should be empty.
        """

    def remove_edge(self, point1, point2):
        """
        Remove the edge of the graph that connects the two given 2D points.

        The points are represented by tuples of two numbers (x, y).

        Has an effect only if both points have a corresponding node in the
        graph and if those nodes are connected by an edge.
        """
        pass

#==============================================================================
