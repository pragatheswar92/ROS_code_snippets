#!/usr/bin/env python
 
 
#=============================== YOUR CODE HERE ===============================
# Instructions: complete the currently empty BugBrain class. A new instance of
#               this class will be created for each new move_to command. The
#               constructor receives the goal specification and the mode of
#               wallfollowing (left (0) or right (1)) that is currently in use.
#               All the remaining functions receive the current position and
#               orientation of the robot.
#
# Hint: you can create a class member variable at any place in your code (not
#       only in __init__) by assigning a value to it, e.g.:
#
#           self.some_member_variable = 2012
#
# Hint: you could use the 'planar' library to avoid implementing geometrical
#       functions that check the distance between a point and a line, or any
#       other helper functions that you need. To use its classes add the
#       following import statements on top of the file:
#
#            from planar import Point, Vec2
#            from planar.c import Line
#            from math import degrees
#
#       As discussed in the lab class, you will need to install the library by
#       executing `sudo pip install planar` in the terminal.
#
# Hint: all the member variables whose names start with 'wp_' (which stands for
#       'waypoint') will be automagically visualized in RViz as points of
#       different colors. Similarly, all the member variables whose names
#       start with 'ln_' (which stands for 'line') will be visualized as lines
#       in RViz. The only restriction is that the objects stored in these
#       variables should indeed be points and lines.
#       The valid points are:
#
#           self.wp_one = (1, 2)
#           self.wp_two = [1, 2]
#           self.wp_three = Point(x, y) # if you are using 'planar'
#
#       The valid lines are (assuming that p1 and p2 are valid points):
#
#           self.ln_one = (p1, p2)
#           self.ln_two = [p1, p2]
#           self.ln_three = Line.from_points([p1, p2]) # if you are using 'planar'
 
 
 
from planar import Point, Vec2
from planar.c import Line
class BugBrain:
 
    TOLERANCE = 0.3
 
    def __init__(self, goal_x, goal_y, side):
        #assigning the goal values to self variables self.goal_x and self.goal_y.
        self.goal_x = goal_x
        self.goal_y = goal_y
        #creating the point for goal position
        self.wp_goal_position = Point(self.goal_x,self.goal_y)
        #creating boolean flag
        self.flag = False
        #creating two lists to store the values of point to keep track on the path
        self.x_list = []
        self.y_list = []
        #creating count variable for goal unreachable state
        self.goal_unreachable_count = 0
        pass
 
    def follow_wall(self, x, y, theta):
        """
        This function is called when the state machine enters the wallfollower
        state.
        """
        self.x1 = x
        self.y1 = y
        #adding the current values of x and  y in the list
        self.x_list.append(x)
        self.y_list.append(y)
        #creating point for current x,y values
        self.wp_current_point = Point(x,y)
        #creating line between current point to goal point
        self.ln_one = Line.from_points([self.wp_goal_position,self.wp_current_point])
        pass
 
    def is_time_to_leave_wall(self, x, y, theta):
         #calculating the distance between the current point to the line
        distance = self.ln_one.distance_to(Point(x,y))
        '''
        check whether the distance between the current point and line and
        return TRUE if the point lies on the line (or) return FALSE if the point away from the line
        '''
        if(abs(distance) < 0.02 and (abs(self.x1 - x)>0.5 or abs(self.y1 - y)>0.5)):
            for a, b in zip(self.x_list, self.y_list):
                if(abs(x - a)<0.5 and abs(y - b)<0.5):
                    if(abs(self.x_list[0] - x)<0.5 and abs(self.y_list[0] - y)<0.5):
                        self.goal_unreachable_count += 1
                    self.flag = False
                    break
                else:
                    self.flag = True
       
        #if the flag zero(point which is unvisited), the add the current point to the list
        if(self.flag == True):
            self.x_list.append(x)
            self.y_list.append(y)
            self.flag = False
            return True
        else:
            return False

 
    def leave_wall(self, x, y, theta):
        """
        This function is called when the state machine leaves the wallfollower
        state.
        """
        print 'leave wall'
        pass
 
    def is_goal_unreachable(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether the goal is unreachable.
        """
        #if the robot cross the same point more than twice, then return TRUE(Goal is unreachable)
        if(self.goal_unreachable_count >2):
            return True
       
        return False
