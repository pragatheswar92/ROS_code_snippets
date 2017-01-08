#!/usr/bin/env python

PACKAGE = 'amr_navigation'
NODE = 'path_executor'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from actionlib import SimpleActionClient, SimpleActionServer
from nav_msgs.msg import Path
from amr_msgs.msg import MoveToAction, MoveToGoal, ExecutePathAction, \
                         ExecutePathFeedback, ExecutePathResult

'''
_authors = Pragatheswar Nagarajan, Ramesh Kumar, Rubanraj Ravichandran
_date = 17.11.2016
_assignment = 7
'''
class PathExecutor:

    def __init__(self):
        """
            Action server
        """
        #creating an action server to handle the request from client(sends path contains collection of poses)
        self._move_to_server = SimpleActionServer(NODE+'/execute_path',
                                                  ExecutePathAction,
                                                  self.execute_cb,
                                                  auto_start=False)
        #starting the action server
        self._move_to_server.start()

        #creating a publisher instance to publish the path message in /path_executor/current_path topic for visualization
        self.path_publisher = rospy.Publisher('/path_executor/current_path',
                                        Path,
                                        queue_size=0)

        rospy.loginfo('Started [PathExecutor] node.')
        self.use_obstacle_avoidance = True
        pass

    def execute_cb(self, goal):
        rospy.loginfo(len(goal.path.poses))
        
        if self.use_obstacle_avoidance:
            SERVER = '/bug2/move_to'
        else:
            SERVER = '/motion_controller/move_to'

        #after receiving the path, publish the path
        self.path_publisher.publish(goal.path)

        #creating the client to send each pose to the server
        self.client = SimpleActionClient(SERVER,MoveToAction)

        self.client.wait_for_server()

        #passing each pose to the server and handling the result from the server
        for pose in goal.path.poses:
            
            self.goal_to_move_to = MoveToGoal()
            
            self.feedback = ExecutePathFeedback()
            self.visited = ExecutePathResult()
            
            self.goal_to_move_to.target_pose = pose
            self.client.send_goal(self.goal_to_move_to,done_cb=self.move_to_done_cb)
            
            rospy.logwarn("goal passed to motion_controller")
            self.client.wait_for_result()
            rospy.loginfo('Before feedback pose')
            
        
        self._move_to_server.set_succeeded(self.visited)
            #client.get_result() 

        pass
    '''
    def action_feedback_cb(self, feedback):
        rospy.logwarn("feedback called")
        #rospy.logwarn(feedback)
    '''

    def move_to_done_cb(self, state, result):
        self.feedback.pose = self.goal_to_move_to.target_pose
        if(self.client.get_state() == 3):
            self.feedback.reached = True
        else:
            self.feedback.reached = False
        #publishing the feedback to the client
        self.visited.visited.append(self.feedback.reached)
        self._move_to_server.publish_feedback(self.feedback)
        pass


if __name__ == '__main__':
    rospy.init_node(NODE)
    pe = PathExecutor()
    rospy.spin()

