#!/usr/bin/env python

PACKAGE = 'amr_navigation'

import math
from velocity_controller import VelocityController, Velocity
from velocity_controller import get_shortest_angle, get_distance
from math import atan2, copysign
class OmniVelocityController(VelocityController):
    def __init__(self, l_max_vel, l_tolerance, a_max_vel, a_tolerance):
        self._l_max_vel = l_max_vel
        self._l_tolerance = l_tolerance
        self._a_max_vel = a_max_vel
        self._a_tolerance = a_tolerance
    def compute_velocity(self, actual_pose):

        # calculate the distance between the current pose and the target pose
        dx = self._target_pose.x - actual_pose.x
        dy = self._target_pose.y - actual_pose.y

        # calculating the linear and angular distance
        linear_dist = get_distance(self._target_pose, actual_pose)
        angular_dist = get_shortest_angle(self._target_pose.theta, actual_pose.theta)

        velocity_angle  = get_shortest_angle(atan2(dy, dx), actual_pose.theta)

        if (abs(linear_dist) < self._l_tolerance and abs(angular_dist) < self._a_tolerance):
            self._linear_complete = True
            self._angular_complete = True
            return Velocity()

        # Computing remaining distances
        vx = self._l_max_vel*math.cos(velocity_angle)
        vy = self._l_max_vel*math.sin(velocity_angle) 

        angular_vel = angular_dist / (linear_dist/self._l_max_vel)
        
        if angular_vel > self._a_max_vel:
            angular_vel = self._a_max_vel
            linear_vel = linear_dist /(angular_dist / angular_vel)
            vx =linear_vel*math.cos(velocity_angle)
            vy = linear_vel*math.sin(velocity_angle)
            
        return Velocity(vx,vy,copysign(angular_vel, angular_dist))