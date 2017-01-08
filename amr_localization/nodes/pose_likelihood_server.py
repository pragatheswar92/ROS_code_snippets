#!/usr/bin/env python

PACKAGE = 'amr_localization'
NODE = 'pose_likelihood_server'

import roslib
roslib.load_manifest(PACKAGE)
import rospy
import tf 
from tf import *
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose2D,TransformStamped
from amr_srvs.srv import GetMultiplePoseLikelihood, GetMultiplePoseLikelihoodResponse, GetNearestOccupiedPointOnBeam, GetNearestOccupiedPointOnBeamRequest, SwitchRanger


class PoseLikelihoodServerNode:
    """
    This is a port of the AMR Python PoseLikelihoodServerNode
    """
    def __init__(self):
        
        rospy.init_node(NODE)
        
        # Wait until SwitchRanger service (and hence stage node) becomes available.
        rospy.loginfo('Waiting for the /switch_ranger service to be advertised...');
        rospy.wait_for_service('/switch_ranger')
        
        try:
            switch_ranger = rospy.ServiceProxy('/switch_ranger', SwitchRanger)
            # Make sure that the hokuyo laser is available and enable them (aka switch on range scanner)
            switch_ranger('scan_front', True)
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)

        """
            Expose GetMultiplePoseLikelihood Service here,
            subscribe for /scan_front,
            create client for /occupancy_query_server/get_nearest_occupied_point_on_beam service

            http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
        """
        
        #initialize global variables
        self._tf = tf.TransformListener()
        self.flag = False
        self.transformation_matrix_list = []
        self.sigma = 0.8
        #self.weight_threshold = 0.001


        #1 Creating GetMultiplePoseLikelihood Service
        getMultiplePose = rospy.Service('pose_likelihood_server/get_pose_likelihood', GetMultiplePoseLikelihood, self.get_pose_likelihood_cb)

        #2 Creating subscriber for scan_front
        self._laser_subscriber = rospy.Subscriber('/scan_front',
                                                  LaserScan,
                                                  self._laser_callback,
                                                  queue_size=5)

        pass

    """
    ============================== YOUR CODE HERE ==============================
    Instructions:   implemenent the pose likelihood server node including a
                    constructor which should create all needed servers, clients,
                    and subscribers, and appropriate callback functions.
                    GetNearestOccupiedPointOnBeam service allows to query
                    multiple beams in one service request. Use this feature to
                    simulate all the laser beams with one service call, otherwise
                    the time spent on communication with the server will be too
                    long.

    Hint: refer to the sources of the previous assignments or to the ROS
          tutorials to see examples of how to create servers, clients, and
          subscribers.
    
    Hint: in the laser callback it is enough to just store the incoming laser
          readings in a class member variable so that they could be accessed
          later while processing a service request.
  
    Hint: the GetNearestOccupiedPointOnBeam service may return arbitrary large
          distance, do not forget to clamp it to [0..max_range] interval.


    Look at the tf library capabilities, you might need it to find transform
    from the /base_link to /base_laser_front_link.
    Here's an example how to use the transform lookup:

        time = self._tf.getLatestCommonTime(frame_id, other_frame_id)
        position, quaternion = self._tf.lookupTransform(frame_id,
                                                        other_frame_id,
                                                        time)
        yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        x, y, yaw = position[0], position[1], yaw

    You might need other functions for transforming routine, you can find
    a brief api description
    http://mirror.umd.edu/roswiki/doc/diamondback/api/tf/html/python/tf_python.html
    m = geometry_msgs.msg.TransformStamped()
    m.header.frame_id = 'base_link'
    m.child_frame_id = 'base_laser_front_link'
    """

    #scan front callback method
    def _laser_callback(self,msg):

        #Storing maximum range of laser and range values
        self.max_range = msg.range_max
        self.range = msg.ranges 
        
        #Transforming laser frame to base frame
        if(self.flag == False):
            _base_link = '/base_link'
            _base_laser_front_link = '/base_laser_front_link'
            self._angle_increment = msg.angle_increment
            self._angle_min = msg.angle_min
            self._tf.waitForTransform(_base_link, _base_laser_front_link, rospy.Time(), rospy.Duration(2))
            
            time = self._tf.getLatestCommonTime(_base_laser_front_link,_base_link)
            
            position, quaternion = self._tf.lookupTransform(_base_link,_base_laser_front_link,time)
            self.yaw_laser = tf.transformations.euler_from_quaternion(quaternion)[2]
            angle = self._angle_min
            self.x_laser, self.y_laser, self.yaw_laser = position[0], position[1], self.yaw_laser
            
            #self.transformation_matrix_list.append(trans)
            #creating transformation matrix for each beam and storing all the transformation matrix in a list
            #for i in range(2, 13):
                #angle = angle + self._angle_increment
                #trans = np.array([[np.cos(angle),-np.sin(angle),self.x_laser ],[np.sin(angle),np.cos(angle),self.y_laser ],[0,0,1]],dtype=float)
            self.flag = True
            '''
            for i in range(1, 13):
                trans = np.array([[np.cos(i*self._angle_increment),-np.sin(i*self._angle_increment),x ],[np.sin(i*self._angle_increment),np.cos(i*self._angle_increment),y ],[0,0,1]],dtype=float)
                self.transformation_matrix_list.append(trans)
                self.flag = True
            '''
    # pose likelihood service call back
    def get_pose_likelihood_cb(self,req):
        counter = 0
        result = []
        
        #iterating poses provided by client and finding the distances
        for pose in req.poses:
            
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            
            x_orient = pose.pose.orientation.x
            y_orient = pose.pose.orientation.y
            z_orient = pose.pose.orientation.z
            w_orient = pose.pose.orientation.w
            #this should  be 3*1
            self.laser_pose_initial = np.array([self.x_laser,self.y_laser,1])

            quaternion = (x_orient,y_orient,z_orient,w_orient)
            
            #Computing yaw from the x,y,z,w orientation using euler_from_quaternion() method
            self.theta_user = tf.transformations.euler_from_quaternion(quaternion)[2]

            current_pose = np.array([[x],[y],[1]])

            pose_list = []
            #Multiplying transformation matrix with the pose which is derived from client request
            for i in range(0,12):
                trans = np.array([[np.cos(self.theta_user),-np.sin(self.theta_user),x ],[np.sin(self.theta_user),np.cos(self.theta_user),y ],[0,0,1]],dtype=float)

                temp = np.dot(trans,self.laser_pose_initial)
                
                pose2d = Pose2D()
                pose2d.x = temp[0]
                pose2d.y = temp[1]
                pose2d.theta = self._angle_min + self.theta_user + i*self._angle_increment
                #creating pose and storing it in a list for all beams
                pose_list.append(pose2d)

            

            
                
            #3 creating client for /occupancy_query_server/get_nearest_occupied_point_on_beam service
            rospy.wait_for_service('/occupancy_query_server/get_nearest_occupied_point_on_beam')

            try:
                get_nearest_occupied_point_on_beam= rospy.ServiceProxy('/occupancy_query_server/get_nearest_occupied_point_on_beam', GetNearestOccupiedPointOnBeam)
                threshold = 2
                request = GetNearestOccupiedPointOnBeam()
                #request.threshold = 2
                #request.beams= pose_list
                #sending the pose list and threshold to occupancy server
                response = get_nearest_occupied_point_on_beam(pose_list,threshold)
                response_distances = np.array(response.distances)
                modified_distance_list = []
                #calculating difference in distance which is given by occupancy server and current distance range from scan_front
                for i in range(len(response_distances)):
                    #before finding the difference, remove the "inf" values in distance by clamping. (replacing "inf" value with max_range value)
                    if(np.isinf(response_distances[i]) or np.isnan(response_distances[i]) or response_distances[i]>self.max_range):
                        modified_distance_list.append(self.max_range)
                    else:
                        modified_distance_list.append(response_distances[i])

                self.errors = list(np.array(modified_distance_list) - np.array(list(self.range)))
                self.errors = map(abs,self.errors)
                
                self.errors.sort()
                self.count_bad_values = 0
                for i in range(len(self.errors)):
                    if(self.errors[i]< (self.sigma*2)):
                        self.count_bad_values = self.count_bad_values + 1
                self.weights = []
                
                if(self.count_bad_values >= 4):
                    self.weights = self.errors
                else:
                    for i in range(self.count_bad_values,len(self.errors)):
                        self.weights.append(self.errors[i])


                #calculating the likelyhood
                self.weights,likelyhood = self.calculate_likelihood(self.weights)
                #if (self.count_bad_values < 4):
                likelyhood = likelyhood * (10000)
                #likelyhood = likelyhood * (self.count_bad_values/12)
                result.append(likelyhood)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        #print len(result)
        #creating object for returning response to client
        resp = GetMultiplePoseLikelihoodResponse()
        #assign the values to the response.likelihoods
        resp.likelihoods = result
        return resp

    #function to calculate likelihood
    def calculate_likelihood(self,weights):
        expression_one = 1 / (self.sigma * math.sqrt(2*math.pi))
        new_weights = []
        likely_hoods = 1
        for i in range(len(weights)):
            expression_two = math.exp(-math.pow(weights[i],2)/2*math.pow(self.sigma,2))
            result = expression_one*expression_two
            new_weights.append(result)
            likely_hoods *= result
        return new_weights,likely_hoods

if __name__ == '__main__':
    w = PoseLikelihoodServerNode()
    rospy.spin()

