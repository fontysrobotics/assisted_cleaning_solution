#!/usr/bin/env python

import roslib; roslib.load_manifest('assisted_cleaning_solution')

import rospy
import actionlib
import tf.transformations
import math

from nav_msgs.msg import Odometry
from assisted_cleaning_solution.msg import MoveAroundChairAction
from util import move

class move_around_chair:
    def __init__(self):
        self.goal_received = False
        self.step = 0
        self.turn_dir = 0
        self.times_rotated = 0
        self.x_odom = None
        self.y_odom = None
        self.yaw_odom = None
        self.yaw_start = None
        self.yaw_first_stop = None

        # location of table legs. So the robot doesn't drive against them
        self.x_table_list = [2.567, 3.421, 2.515, 5.254]
        self.y_table_list = [4.215, 4.215, 1.871, 1.871]

    def subs_chairs(self):
        self.server_move_around_chair = actionlib.SimpleActionServer('move_around_chair_action', 
            MoveAroundChairAction, 
            auto_start = False)

        self.server_move_around_chair.register_goal_callback(self.goalCB)
        self.server_move_around_chair.register_preempt_callback(self.preemptCB)
        
        self.server_move_around_chair.start()

        rospy.Subscriber('odom', Odometry, self.odomCB)
        
    def goalCB(self):
        self.chairs_coords = self.server_move_around_chair.accept_new_goal()

        self.turn_dir = 0
        self.goal_received = True

    def preemptCB(self):
        move(0,0)
        self.goal_received = False
        self.server_move_around_chair.set_preempted()

    def odomCB(self, message):
        quat = message.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([0, 0, quat.z, quat.w])

        self.x_odom = message.pose.pose.position.x
        self.y_odom = message.pose.pose.position.y
        self.yaw_odom = euler[2]

        self.chair()

    def chair(self):
        if (self.goal_received == True):
            # Start with calculating where to stop
            if (self.step == 0):
                self.yaw_start = self.yaw_odom
                self.yaw_first_stop = self.yaw_start + 3.14

                if self.yaw_first_stop > 3.14:
                    self.yaw_first_stop = self.yaw_first_stop - 6.28

                if self.yaw_first_stop < -3.14:
                    self.yaw_first_stop = self.yaw_first_stop + 6.28

                self.step = 1

            else:
                # Decide if robot is to close to a table leg to then move backward
                for index in range(len(self.x_table_list)):
                    dis_to = math.hypot(self.x_table_list[index]-self.x_odom, self.y_table_list[index]-self.y_odom)
                    if (dis_to < 0.45):
                        if (self.turn_dir == 0 and self.times_rotated == 0):
                            self.turn_dir = 1
                            self.times_rotated = 1

                        if (self.turn_dir == 1 and self.times_rotated == 0):
                            self.turn_dir = 0
                            self.times_rotated = 1

                # Direction to drive
                if (self.turn_dir == 0):
                    move(0.19, 0.4)

                if (self.turn_dir == 1):
                    move(-0.19, -0.4)

                # Decide when to stop the first time
                if (self.step == 1 and (self.yaw_odom > self.yaw_first_stop - 0.2) and (self.yaw_odom < self.yaw_first_stop + 0.2)):
                    print('first stop')
                    move(0, 0)
                    rospy.sleep(2)
                    self.step = 2
                    self.goal_received = False
                    self.server_move_around_chair.set_succeeded()

                # Decide when to stop the second time
                if (self.step == 2 and (self.yaw_odom > self.yaw_start - 0.2) and (self.yaw_odom < self.yaw_start + 0.2)):
                    print('second stop')
                    move(0, 0)
                    rospy.sleep(2)
                    self.step = 0
                    self.times_rotated = 0
                    self.goal_received = False
                    self.server_move_around_chair.set_succeeded()


if __name__ == "__main__":
    rospy.init_node('move_around_chair')
    chairs = move_around_chair()
    chairs.subs_chairs()
    rospy.spin()