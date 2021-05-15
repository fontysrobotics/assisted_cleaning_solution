#!/usr/bin/env python

import roslib; roslib.load_manifest('assisted_cleaning_solution')

import rospy
import actionlib
import message_filters
import math

from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from util import move

from assisted_cleaning_solution.msg import CleanPlinthsAction, CleanPlinthsResult

class followPlinths():
    _result = CleanPlinthsResult()

    def __init__(self):
        self.sonar_front = None
        self.sonar_back = None
        self.ir_front = None
        self.ir_back = None

        self.coords_room = None
        self.dis_to_start = None
        self.odom_x = None
        self.odom_y = None

        self.position = 1

        self.close_to_wall = 0.10
        self.far_to_wall = 0.13

    def subs_plinths(self):
        self.server_plinths = actionlib.SimpleActionServer('follow_plinths_action', 
            CleanPlinthsAction, 
            auto_start = False)

        self.server_plinths.register_goal_callback(self.goalCB)
        self.server_plinths.register_preempt_callback(self.preemptCB)
        
        self.server_plinths.start()

        sub_sonar1 = message_filters.Subscriber('sonar1', Range)
        sub_sonar2 = message_filters.Subscriber('sonar2', Range)
        sub_ir_front = message_filters.Subscriber('ir_front', Range)
        sub_ir_back = message_filters.Subscriber('ir_back', Range)
        sub_odom = message_filters.Subscriber('odom', Odometry)
        
        ts = message_filters.ApproximateTimeSynchronizer([sub_sonar1, sub_sonar2, sub_ir_front, sub_ir_back, sub_odom], 10, 0.1)
        ts.registerCallback(self.sensorsCB)   

    def goalCB(self):
        self.coords_room = self.server_plinths.accept_new_goal()
        self.coords_room = self.coords_room.coords
        self.position = 0

    def preemptCB(self):
        move(0,0)
        self.position = 1
        self.server_plinths.set_preempted()
        
    def sensorsCB(self, sonar1, sonar2, ir_front, ir_back, odom):
        self.sonar_back = sonar1.range
        self.sonar_front = sonar2.range
        self.ir_front = ir_front.range
        self.ir_back = ir_back.range

        self.odom_x = odom.pose.pose.position.x
        self.odom_y = odom.pose.pose.position.y

        self.situation()

    def situation(self):
        # Only work when goal is received
        if (self.position == 0):

            # Drive forward when nothing at robot side, This is for the robot reaching door
            if (self.sonar_front > 1.5):
                move(0.4, 0)
            
            # Only do this when there is no open space on right side of robot
            else:
                # Calculate if robot is close to end of room
                self.dis_to_start = math.hypot(self.coords_room[0] - self.odom_x, self.coords_room[1] - self.odom_y)

                # stop the robot when the robot is back to begin position
                if (self.dis_to_start < 0.3):    
                    move(0,0)
                    self.position = 1

                    self._result.done = False
                    self.server_plinths.set_succeeded(self._result)  

                # When the robot detects something in front
                elif (self.ir_front < 0.13):
                    move(0,0)
                    self.position = 1

                    self._result.done = True
                    self.server_plinths.set_succeeded(self._result)

                # Follow the wall
                else:
                    if (self.sonar_front > 0.20):
                        move(0.02, -1.2)

                    else:
                        self.follow_side()

    def follow_side(self):
        # The function for following the wall and correcting robot orientation
        if (self.sonar_front > self.far_to_wall):
            move(0.1, -0.2)
        
        elif (self.sonar_front < self.close_to_wall):
            move(0.1, 0.2)
        
        else:
            if (self.sonar_back > self.far_to_wall):
                move(0.1, 0.2)

            elif (self.sonar_back < self.close_to_wall):
                move(0.1, -0.2)

            else:
                move(0.5, 0)

if __name__ == "__main__":
    rospy.init_node('follow_plinths')
    server_plinths = followPlinths()
    server_plinths.subs_plinths()
    rospy.spin()

