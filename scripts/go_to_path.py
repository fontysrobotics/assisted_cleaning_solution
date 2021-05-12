#!/usr/bin/env python

import roslib; roslib.load_manifest('assisted_cleaning_solution')

import rospy
import actionlib
import tf.transformations
import math

from nav_msgs.msg import Odometry
from assisted_cleaning_solution.msg import GoToPathAction
from util import move

class goToPath:
    def __init__(self):
        self.goal_received = False
        self.x_odom = None
        self.y_odom = None
        self.yaw_odom = None

    def subs_path(self):
        self.server_go_to_path = actionlib.SimpleActionServer('go_to_path_action', 
            GoToPathAction, 
            auto_start = False)

        self.server_go_to_path.register_goal_callback(self.goalCB)
        self.server_go_to_path.register_preempt_callback(self.preemptCB)
        
        self.server_go_to_path.start()

        rospy.Subscriber('odom', Odometry, self.odomCB)
        
    def odomCB(self, message):
        quat = message.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([0, 0, quat.z, quat.w])

        self.x_odom = message.pose.pose.position.x
        self.y_odom = message.pose.pose.position.y
        self.yaw_odom = euler[2]

        self.path()

    def goalCB(self):
        self.pose_coords = self.server_go_to_path.accept_new_goal()

        self.x_coord = self.pose_coords.pose_around_chair[0]
        self.y_coord = self.pose_coords.pose_around_chair[1]
        
        self.goal_received = True

    def preemptCB(self):
        move(0,0)
        self.goal_received = False
        self.server_go_to_path.set_preempted()
        
    def path(self):
        if (self.goal_received == True):
            extra = 0.05
            goal_dir = math.atan2(self.y_coord - self.y_odom, self.x_coord - self.x_odom)
            dis_to = math.hypot(self.x_coord - self.x_odom, self.y_coord - self.y_odom)

            # Turn to point in path
            if (self.yaw_odom > goal_dir + extra):
                move(0, -0.5)

            if (self.yaw_odom < goal_dir - extra): 
                move(0, 0.5)

            # Go to point in path
            if ((dis_to > extra) and (self.yaw_odom > goal_dir - extra) and (self.yaw_odom < goal_dir + extra)):
                move(0.4, 0)
            
            # Stop when robot is back in place
            if ((dis_to > -extra) and (dis_to < extra)):
                move(0, 0)
                self.goal_received = False
                self.server_go_to_path.set_succeeded()
            
if __name__ == "__main__":
    rospy.init_node('go_to_path')
    path = goToPath()
    path.subs_path()
    rospy.spin()