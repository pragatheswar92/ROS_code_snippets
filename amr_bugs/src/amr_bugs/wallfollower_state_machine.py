#!/usr/bin/env python

"""
This module provides a single construct() function which produces a Smach state
machine that implements wallfollowing behavior.

The state machine contains three states:
    * findWall:     initial state - drives until a wall is detected
    * alignWall     aligning state - used to align at convex corners or walls in front of robot 
    * followWall    following state - used to follow a straight wall, robust against sensor noise, curls around concave corners

The constructed state machine has three attached methods:
    * set_ranges(ranges): this function should be called to update the range
                          readings
    * get_twist(): returns a twist message that could be directly passed to the
                   velocity publisher
    * set_config(config): updates the machine userdata with the new config

The constructed state machine is preemptable, i.e. each state checks whether
a preemption is requested and returns 'preempted' if that is the case.
"""

PACKAGE = 'amr_bugs'

import roslib
roslib.load_manifest(PACKAGE)
import smach
import rospy
from preemptable_state import PreemptableState
from math import log
from math import cos
from math import pi
from types import MethodType
from geometry_msgs.msg import Twist

import time


__all__ = ['construct']
rospy.logwarn('constructing...')
#=============================== YOUR CODE HERE ===============================
# Instructions: write a function for each state of wallfollower state machine.
#               The function should have exactly one argument (userdata
#               dictionary), which you should use to access the input ranges
#               and to provide the output velocity.
#               The function should have at least one 'return' statement, which
#               returns one of the possible outcomes of the state.
#               The function should not block (i.e. have infinite loops), but
#               rather it should implement just one iteration (check
#               conditions, compute velocity), because it will be called
#               regularly from the state machine.
#
# Hint: below is an example of a state that moves the robot forward until the
#       front sonar readings are less than the desired clearance. It assumes
#       that the corresponding variables ('front_min' and 'clearance') are
#       available in the userdata dictionary.
#
#           def search(ud):
#               if ud.front_min < ud.clearance:
#                   return 'found_obstacle'
#               ud.velocity = (1, 0, 0)
#==============================================================================

'''
find_wall state is used to find the wall
'''
def find_wall(ud):
    rospy.logwarn('find_wall if...')
    if ud.mode == 1:
        if ud.front_min < ud.clearance:
            ud.velocity = (0,0,0)
            return 'wall_found'
    ud.velocity = (ud.max_forward_velocity,0,0)
    if ud.mode == 0:
        if ud.front_min < ud.clearance:
            ud.velocity = (0,0,0)
            return 'wall_found'
    ud.velocity = (ud.max_forward_velocity,0,0)

#turn state
def turn(ud):
    rospy.logwarn('turn if...')
    if ud.mode == 1:
        if round(ud.ranges[8].range,2) <= ud.clearance:
            if round(ud.ranges[7].range,2) == round(ud.ranges[8].range,2):
                ud.velocity = (0,0,0)
                return 'alligned'
    ud.velocity = (0.01,0,0.3)
    if ud.mode == 0:
        if round(ud.ranges[15].range,2) <= ud.clearance:
            if round(ud.ranges[0].range,2) == round(ud.ranges[15].range,2):
                ud.velocity = (0,0,0)
                return 'alligned'
    ud.velocity = (0.01,0,-0.3)
    
#align state
def align(ud):
    if ud.mode == 1:     
        if ud.front_min <= ud.clearance:
            ud.velocity = (0.3,0,0.3)
        if round(ud.ranges[7].range,2) > round(ud.ranges[8].range,2):
            ud.velocity = (0.1,0,-0.5)
        elif round(ud.ranges[7].range,2) < round(ud.ranges[8].range,2):
            ud.velocity = (0.1,0,0.5)
        elif round(ud.ranges[7].range,2) == round(ud.ranges[8].range,2):
                ud.velocity = (0,0,0)
                return 'alligned'
    if ud.mode == 0:      
        if ud.front_min <= ud.clearance:
            ud.velocity = (0.3,0,-0.3)
        if round(ud.ranges[0].range,2) > round(ud.ranges[15].range,2):
            ud.velocity = (0.1,0,0.5)
        elif round(ud.ranges[0].range,2) < round(ud.ranges[15].range,2):
            ud.velocity = (0.1,0,-0.5)
        elif round(ud.ranges[0].range,2) == round(ud.ranges[15].range,2):
                ud.velocity = (0,0,0)
                return 'alligned'

#follow_wall state
def follow_wall(ud):
    if ud.mode == 1:
        if ud.front_min <= ud.clearance:
            ud.velocity = (0.3,0,0.3)        
        if ud.ranges[7].range > 1:
            return 'door_found'
        else:
            if round(min(ud.ranges[3].range,ud.ranges[4].range,ud.ranges[5].range),2) <= round(ud.clearance,2) and ud.right_min <= round(ud.clearance,2):
                ud.velocity = (0,0,0)
                return 'corner_found'
            if round(ud.ranges[7].range,2) != round(ud.ranges[8].range,2):
                return 'not_aligned'
        ud.velocity = (ud.max_forward_velocity,0,0)
    if ud.mode == 0: 
        if ud.front_min <= ud.clearance:
            ud.velocity = (0.3,0,-0.3)
        if ud.ranges[0].range > 1:
            return 'door_found'
        else:
            if round(min(ud.ranges[3].range,ud.ranges[4].range,ud.ranges[5].range),2) <= ud.clearance and ud.left_min <= ud.clearance:
                ud.velocity = (0,0,0)
                return 'corner_found'
            if round(ud.ranges[0].range,2) != round(ud.ranges[15].range,2):
                return 'not_aligned'
        ud.velocity = (ud.max_forward_velocity,0,0)

#corner handle state
def corner_handle(ud):
    if ud.mode == 1:
        min_b = round(ud.ranges[13].range,2)
        if min_b <= ud.clearance:
            ud.velocity = (0,0,0)
            return 'alligned'
    ud.velocity=(0.02,0,0.3)
    if ud.mode == 0:
        min_b = round(ud.ranges[10].range,2)
        if min_b <= ud.clearance:
            ud.velocity = (0,0,0)
            return 'alligned'
    ud.velocity=(0.02,0,-0.3)

#door handle state    
def door_handle(ud):
    if ud.mode == 1:        
        if round(ud.ranges[7].range,2) < ud.clearance or round(min(ud.ranges[4].range,ud.ranges[5].range,ud.ranges[6].range),2) < ud.clearance:
            return 'alligned'
    ud.velocity = (ud.max_forward_velocity,0,-0.4)
    if ud.mode == 0:       
        if round(ud.ranges[0].range,2) < ud.clearance or round(min(ud.ranges[1].range,ud.ranges[2].range,ud.ranges[3].range),2) < ud.clearance:
            return 'alligned'
    ud.velocity = (ud.max_forward_velocity,0,0.4)
def set_ranges(self, ranges):
    """
    This function will be attached to the constructed wallfollower machine.
    Its argument is a list of Range messages as received by a sonar callback. 
    For left hand side wallfollowing, the sensor values are mirrored (sides are swapped).
    """


    #============================= YOUR CODE HERE =============================
    # Instructions: store the ranges from a ROS message into the userdata
    #               dictionary of the state machine.
    #               'ranges' is a list or Range messages (that should be
    #               familiar to you by now). It implies that to access the
    #               actual range reading of, say, sonar number 3, you need to
    #               write:
    #
    #                   ranges[3].range
    #
    #               For example, to create an item called 'front_min', which
    #               contains the minimum between the ranges reported by the two
    #               front sonars, you would write the following:
    #
    #                   self.userdata.front_min = min(ranges[3].range, ranges[4].range)
    #
    # Hint: you can just store the whole array of the range readings, but to
    #       simplify the code in your state functions, you may compute
    #       additional values, e.g. the difference between the reading of the
    #       side sonars, or the minimum of all sonar readings, etc.
    #
    # Hint: you can access all the variables stored in userdata. This includes
    #       the current settings of the wallfollower (that is clearance and the
    #       mode of wallfollowing). Think about how you could make your state
    #       functions independent of wallfollowing mode by smart preprocessing
    #       of the sonar readings.
    #==========================================================================
    self.userdata.front_min = round(min(ranges[1].range,ranges[2].range,ranges[3].range,ranges[4].range,ranges[5].range,ranges[6].range),2)
    self.userdata.back_min = round(min(ranges[10].range, ranges[11].range,ranges[12].range, ranges[13].range),2)
    self.userdata.right_min = round(min(ranges[7].range, ranges[8].range),2)
    self.userdata.left_min = round(min(ranges[0].range, ranges[15].range),2)
    self.userdata.ranges = ranges

def get_twist(self):
    """
    This function will be attached to the constructed wallfollower machine.
    It creates a Twist message that could be directly published by a velocity
    publisher. The values for the velocity components are fetched from the
    machine userdata.
    """
    twist = Twist()
    twist.linear.x = self.userdata.velocity[0]
    twist.linear.y = self.userdata.velocity[1]
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = self.userdata.velocity[2]

    #============================= YOUR CODE HERE =============================
    # Instructions: although this function is implemented, you may need to
    #               slightly tweak it if you decided to handle wallfolllowing
    #               mode in "the smart way".
    # Hint: state machine userdata is accessible in this function as well, for
    #       example you can read the current wallfollowing mode with
    #
    #           self.userdata.mode
    #
    #==========================================================================

    return twist


def set_config(self, config):
    """
    This function will be attached to the constructed wallfollower machine.
    It updates the relevant fields in the machine userdata.
    Its argument is the config object that comes from ROS dynamic reconfigure
    client. self.userdata.direction sets a velocity sign depending on the mode.
    """
    self.userdata.mode = config['mode']
    self.userdata.clearance = config['clearance']
    if self.userdata.mode == 1:
        self.userdata.direction = 1
    else:
        self.userdata.direction = -1
    return config


def construct():
    rospy.logwarn('align if...')
    sm = smach.StateMachine(outcomes=['preempted'])
    # Attach helper functions
    sm.set_ranges = MethodType(set_ranges, sm, sm.__class__)
    sm.get_twist = MethodType(get_twist, sm, sm.__class__)
    sm.set_config = MethodType(set_config, sm, sm.__class__)
    # Set initial values in userdata
    sm.userdata.velocity = (0, 0, 0)
    sm.userdata.mode = 1
    sm.userdata.clearance = 0.3
    sm.userdata.ranges = None
    sm.userdata.max_forward_velocity = 0.3
    sm.userdata.default_rotational_speed = 0.5
    sm.userdata.direction = 1
    
    with sm:
        pass
        #=========================== YOUR CODE HERE ===========================
        # Instructions: construct the state machine by adding the states that
        #               you have implemented.
        #               Below is an example how to add a state:
        #
        #                   smach.StateMachine.add('SEARCH',
        #                                          PreemptableState(search,
        #                                                           input_keys=['front_min', 'clearance'],
        #                                                           output_keys=['velocity'],
        #                                                           outcomes=['found_obstacle']),
        #                                          transitions={'found_obstacle': 'ANOTHER_STATE'})
        #
        #               First argument is the state label, an arbitrary string
        #               (by convention should be uppercase). Second argument is
        #               an object that implements the state. In our case an
        #               instance of the helper class PreemptableState is
        #               created, and the state function in passed. Moreover,
        #               we have to specify which keys in the userdata the
        #               function will need to access for reading (input_keys)
        #               and for writing (output_keys), and the list of possible
        #               outcomes of the state. Finally, the transitions are
        #               specified. Normally you would have one transition per
        #               state outcome.
        #
        # Note: The first state that you add will become the initial state of
        #       the state machine.
        #======================================================================
        smach.StateMachine.add('FINDWALL',PreemptableState(find_wall,input_keys=['mode','front_min', 'clearance','right_min','ranges','max_forward_velocity'],output_keys=['velocity'],outcomes=['wall_found']),transitions={'wall_found': 'TURN'})
        smach.StateMachine.add('TURN',PreemptableState(turn,input_keys=['mode','front_min', 'clearance','ranges','right_min','max_forward_velocity'],output_keys=['velocity'],outcomes=['alligned','find_wall']),transitions={'alligned': 'FOLLOW','find_wall':'FINDWALL'})
        smach.StateMachine.add('ALIGN',PreemptableState(align,input_keys=['mode','front_min', 'clearance','ranges','right_min','max_forward_velocity'],output_keys=['velocity'],outcomes=['alligned','not_aligned','find_wall']),transitions={'alligned': 'FOLLOW','not_aligned':'TURN','find_wall':'FINDWALL'})
        smach.StateMachine.add('FOLLOW',PreemptableState(follow_wall,input_keys=['mode','front_min', 'clearance','ranges','right_min','left_min','max_forward_velocity'],output_keys=['velocity'],outcomes=['corner_found','door_found','not_aligned','find_wall']),transitions={'corner_found': 'CORNERS','door_found': 'DOOR','not_aligned': 'ALIGN','find_wall':'FINDWALL'})
        smach.StateMachine.add('CORNERS',PreemptableState(corner_handle,input_keys=['mode','back_min', 'clearance','ranges','right_min','max_forward_velocity'],output_keys=['velocity'],outcomes=['alligned','find_wall']),transitions={'alligned': 'ALIGN','find_wall':'FINDWALL'})                      
        smach.StateMachine.add('DOOR',PreemptableState(door_handle,input_keys=['mode','front_min','back_min', 'clearance','ranges','right_min','max_forward_velocity'],output_keys=['velocity'],outcomes=['alligned','find_wall']),transitions={'alligned': 'ALIGN','find_wall':'FINDWALL'})                      
    return sm
