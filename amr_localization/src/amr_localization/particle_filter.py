#!/usr/bin/env python

PACKAGE = 'amr_localization'
NODE = 'particle_filter'

import roslib
roslib.load_manifest(PACKAGE)
import rospy
import math
import random
from amr_localization.motion_model import MotionModel
from amr_localization.pose import Pose
from amr_localization.particle import Particle
from amr_localization.random_particle_generator import RandomParticleGenerator
import numpy as np

class ParticleFilter:

    def __init__(self, map_min_x, map_max_x, map_min_y, map_max_y, weigh_particles_callback):
        self.weigh_particles_callback = weigh_particles_callback
        self.particle_set_size = 100
        self.random_particles_size = 10
        self.motion_model = MotionModel(0.02, 0.01)
        self.random_particle_generator = RandomParticleGenerator(map_min_x, map_max_x, map_min_y, map_max_y)

        self.particles = []
        for i in range(self.particle_set_size):
            self.particles.append(self.random_particle_generator.generate_particle())


    def update(self, x, y, yaw):

        self.particles_list = []

        #setting the parameters(given by odometry) to motion model
        self.motion_model.setMotion(x,y,yaw)

        #Sampling for all randomly generated particles and storing the result in particle list
        for i in range(self.particle_set_size):
            p = Particle() 
            p.pose = self.motion_model.sample(self.particles[i].pose)
            self.particles_list.append(p)

        #Calculating weights for sampled particles
        weights = self.weigh_particles_callback(self.particles_list)
        for i in range(len(weights)):
            self.particles_list[i].weight = weights[i]

        #Re-sampling using "Stochastic universal re-sampling algorithm"
        index = int(random.random()*self.particle_set_size )
        beta = 0
        max_weight = max(weights)

        result = []
        for i in range(self.particle_set_size ):
            beta += random.random() * 0.5* max_weight

            while beta > weights[index]:
                beta -= weights[index]
                index = (index+1)% self.particle_set_size

            p = Particle()
            p.pose.x = random.gauss(self.particles_list[index].pose.x, 0.1)
            p.pose.y = random.gauss(self.particles_list[index].pose.y, 0.1)
            p.pose.theta = random.gauss(self.particles_list[index].pose.theta, 0.8)
            result.append(p) 

        #Computing weights for new particles
        weights_new = self.weigh_particles_callback(result)

        #Normalizing the weights
        c_sum = sum(weights_new)
        for i in range(len(result)):
            result[i].weight = weights_new[i] / c_sum
        
        #Finding the best particle(Particle with maximum weight)
        self.particles = result
        max_index = np.argmax([particle.weight for particle in result])

        #Assigning the best pose estimate
        self.pose_estimate = result[max_index].pose

            


        '''
        ============================== YOUR CODE HERE ==============================
         Instructions: do one complete update of the particle filter. It should
                       generate new particle set based on the given motion
                       parameters and the old particle set, compute the weights of
                       the particles, resample the set, and update the private
                       member field that holds the current best pose estimate
                       (self.pose_estimate). Note that the motion parameters provided
                       to this function are in robot's reference frame.

         Hint: Check motion_model.py for documentation and example how to use it
               for the particle pose update.
               x, y, yaw -- are the odometry update of the robot's pose (increments)


         Remark: to compute the weight of particles, use the weigh_particles_callback member
                 field:

                     weights = self.weigh_particles_callback(particles_list)

         Remark: to generate a comletely random particle use the provided
                 random_particle_generator object:

                     particle = self.random_particle_generator.generate_particle()

         Finally  the best pose estimate and assign it to the corresponding member field
         for visualization:
                     self.pose_estimate = selected_particle.pose
        ============================================================================
        '''
        pass


    def get_particles(self):
        return self.particles

    def get_pose_estimate(self):
        return self.pose_estimate

    def set_external_pose_estimate(self, pose):
        self.random_particle_generator.set_bias(pose, 0.5, 500)