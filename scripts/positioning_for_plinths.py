#!/usr/bin/env python

import roslib; roslib.load_manifest('assisted_cleaning_solution')

import rospy
import actionlib
import message_filters
import math

from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from util import move

from assisted_cleaning_solution.msg import PositioningPlinthsAction

class positioningForPlinths():
    def __init__(self):
        self.sonar_front = None
        self.sonar_back = None

        self.odom_x = None
        self.odom_y = None
        self.begin_odom_x = None
        self.begin_odom_y = None

        self.position = -1

        self.close_to_wall = 0.10
        self.far_to_wall_positioning = 0.15

    def subs_plinths(self):
        self.server_plinths = actionlib.SimpleActionServer('positioning_for_plinths_action', 
            PositioningPlinthsAction, 
            auto_start = False)

        self.server_plinths.register_goal_callback(self.goalCB)
        self.server_plinths.register_preempt_callback(self.preemptCB)
        
        self.server_plinths.start()

        sub_sonar1 = message_filters.Subscriber('sonar1', Range)
        sub_sonar2 = message_filters.Subscriber('sonar2', Range)
        sub_odom = message_filters.Subscriber('odom', Odometry)
        
        ts = message_filters.ApproximateTimeSynchronizer([sub_sonar1, sub_sonar2, sub_odom], 10, 0.1)
        ts.registerCallback(self.sensorsCB)   

    def goalCB(self):
        self.position = 0
        self.server_plinths.accept_new_goal()

    def preemptCB(self):
        move(0,0)
        self.position = -1

        self.server_plinths.set_preempted()
        
    def sensorsCB(self, sonar1, sonar2, odom):
        self.sonar_back = sonar1.range
        self.sonar_front = sonar2.range

        self.odom_x = odom.pose.pose.position.x
        self.odom_y = odom.pose.pose.position.y

        self.positioning()

    def positioning(self):
        # Starting point next to wall
        if (self.position == 0):
            self.begin_odom_x = self.odom_x
            self.begin_odom_y = self.odom_y
            self.position = 1

        # Move front of robot closer to wall
        if (self.position == 1):
            if (self.sonar_front > self.far_to_wall_positioning):
                move(0.1, -0.1)
            
            if (self.sonar_front < self.close_to_wall):
                move(0.1, 0.1)

            if (self.sonar_front > self.close_to_wall and self.sonar_front < self.far_to_wall_positioning):
                self.position = 2

        # Move back of robot closer to wall
        if (self.position == 2):
            if (self.sonar_back > 0.13):
                move(0.06, 0.2)

            if (self.sonar_back < self.close_to_wall):
                move(0.06, -0.2)

            if (self.sonar_back < 0.13 and self.sonar_back > self.close_to_wall):
                self.position = 3

        # Move back to starting point
        if (self.position == 3):
            move(-0.3, 0)
            self.dis_to_start = math.hypot(self.begin_odom_x - self.odom_x, self.begin_odom_y - self.odom_y)

            if (self.dis_to_start > 0.4 - 0.05 and self.dis_to_start < 0.4 + 0.1):
                move(0, 0)
                self.position = -1
                self.server_plinths.set_succeeded()
            

if __name__ == "__main__":
    rospy.init_node('positioning_for_plinths')
    server_plinths = positioningForPlinths()
    server_plinths.subs_plinths()
    rospy.spin()


