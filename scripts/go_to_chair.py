#!/usr/bin/env python

import roslib; roslib.load_manifest('assisted_cleaning_solution')

import rospy
import actionlib
import tf.transformations
import math

from nav_msgs.msg import Odometry
from assisted_cleaning_solution.msg import GoToChairAction
from util import move

class goToChairs:
    def __init__(self):
        self.goal_received = False
        self.start = 0
        self.x_odom = None
        self.y_odom = None
        self.yaw_odom = None
        self.yaw_parrallel_chair = None

    def subs_chairs(self):
        self.server_go_to_chairs = actionlib.SimpleActionServer('go_to_chair_action', 
            GoToChairAction, 
            auto_start = False)

        self.server_go_to_chairs.register_goal_callback(self.goalCB)
        self.server_go_to_chairs.register_preempt_callback(self.preemptCB)
        
        self.server_go_to_chairs.start()

        rospy.Subscriber('odom', Odometry, self.odomCB)
        
    def odomCB(self, message):
        quat = message.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([0, 0, quat.z, quat.w])

        self.x_odom = message.pose.pose.position.x
        self.y_odom = message.pose.pose.position.y
        self.yaw_odom = euler[2]

        self.chair()

    def goalCB(self):
        self.chairs_coords = self.server_go_to_chairs.accept_new_goal()

        self.x_coord_chair = self.chairs_coords.x_coord_chair
        self.y_coord_chair = self.chairs_coords.y_coord_chair

        self.goal_received = True
        self.start = 0

    def preemptCB(self):
        move(0,0)
        self.goal_received = False
        self.start = 0
        self.server_go_to_chairs.set_preempted()
        
    def chair(self):
        if (self.goal_received == True):
            extra = 0.05
            goal_dir = math.atan2(self.y_coord_chair - self.y_odom, self.x_coord_chair - self.x_odom)
            dis_to = math.hypot(self.x_coord_chair - self.x_odom, self.y_coord_chair - self.y_odom)

            # Rotate to chair point
            if (self.yaw_odom > goal_dir + extra):
                move(0, -0.5)

            if (self.yaw_odom < goal_dir - extra):
                move(0, 0.5)

            # Move to chair
            if (dis_to > 0.71 + extra) and (self.yaw_odom > goal_dir - extra) and (self.yaw_odom < goal_dir + extra):
                move(0.4, 0)
            
            # Rotate till the left side of robot is next to chair
            if (dis_to > 0.71 - extra) and (dis_to < 0.71 + extra):
                # first decide what angle the robot needs to turn to
                if (self.start == 0):
                    self.yaw_parrallel_chair = self.yaw_odom - 1.8

                    if self.yaw_parrallel_chair > 3.14:
                        self.yaw_parrallel_chair = self.yaw_parrallel_chair - 6.28

                    if self.yaw_parrallel_chair < -3.14:
                        self.yaw_parrallel_chair = self.yaw_parrallel_chair + 6.28

                    self.start = 1
                
                # rotate robot till angle reached
                else:
                    move(0, -0.5)

                    if (self.yaw_odom > self.yaw_parrallel_chair - extra) and (self.yaw_odom < self.yaw_parrallel_chair + extra):
                        move(0, 0)
                        self.goal_received = False
                        self.start = 0
                        self.server_go_to_chairs.set_succeeded()

if __name__ == "__main__":
    rospy.init_node('go_to_chairs')
    chairs = goToChairs()
    chairs.subs_chairs()
    rospy.spin()