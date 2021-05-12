#!/usr/bin/env python

import rospy
import actionlib
import message_filters
import tf.transformations

from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from util import move

from assisted_cleaning_solution.msg import MoveAroundObstacleAction

class makeTurn:
    def __init__(self):
        self.sonar_front = None
        self.ir_front = None
        self.ir_back = None

        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.corner_odom_x = None
        self.corner_odom_y = None
        self.corner_odom_yaw = None    

        self.turn_step = 0

    def subs_turning(self):
        self.server_turning = actionlib.SimpleActionServer('move_around_obstacle_action', 
            MoveAroundObstacleAction, 
            auto_start = False)

        self.server_turning.register_goal_callback(self.goalCB)
        self.server_turning.register_preempt_callback(self.preemptCB)
        
        self.server_turning.start()

        sub_sonar2 = message_filters.Subscriber('sonar2', Range)
        sub_ir_front = message_filters.Subscriber('ir_front', Range)
        sub_ir_back = message_filters.Subscriber('ir_back', Range)
        sub_odom = message_filters.Subscriber('odom', Odometry)
        
        ts = message_filters.ApproximateTimeSynchronizer([sub_sonar2, sub_ir_front, sub_ir_back, sub_odom], 10, 0.1)
        ts.registerCallback(self.sensorsCB)   

    def goalCB(self):
        self.turn_step = 1
        self.server_turning.accept_new_goal()

    def preemptCB(self):
        move(0,0)
        self.turn_step = 0

        self.server_turning.set_preempted()
        
    def sensorsCB(self, sonar2, ir_front, ir_back, odom):
        self.sonar_front = sonar2.range
        self.ir_front = ir_front.range
        self.ir_back = ir_back.range

        self.odom_x = odom.pose.pose.position.x
        self.odom_y = odom.pose.pose.position.y

        quat = odom.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([0, 0, quat.z, quat.w])
        self.odom_yaw = euler[2]

        self.turn()

    def turn(self):
        # Start with standing parallel to wall
        if (self.turn_step == 1):
            self.corner_odom_x = self.odom_x
            self.corner_odom_y = self.odom_y
            self.corner_odom_yaw = self.odom_yaw + 1.570796

            move(-0.02, -0.3)

            if (self.corner_odom_yaw > 3.14):
                self.corner_odom_yaw = self.corner_odom_yaw - 6.283185
            
            if (self.sonar_front < 0.15):
                move(0,0)
                self.turn_step = 2

        # Move back to have enough space to make turn
        if (self.turn_step == 2):
            move(-0.4, 0)

            if (self.odom_x < (self.corner_odom_x - 0.15) or self.odom_x > (self.corner_odom_x + 0.15) or self.odom_y < (self.corner_odom_y - 0.15) or self.odom_y > (self.corner_odom_y + 0.15)):
                move(0,0)
                self.turn_step = 3

        # Make turn
        if (self.turn_step == 3):
            move(0.16, 0.8)
            
            if (self.odom_yaw > (self.corner_odom_yaw - 0.05) and self.odom_yaw < (self.corner_odom_yaw + 0.05)):
                move(0,0)
                self.turn_step = 4

        # Drive back to clean corners
        if (self.turn_step == 4):
            move(-0.2, 0)
            
            if (self.ir_back < 0.10):
                move(0,0)
                self.turn_step = 0
                self.server_turning.set_succeeded()

if __name__ == "__main__":
    rospy.init_node('move_around_obstacle')
    server_turning = makeTurn()
    server_turning.subs_turning()
    rospy.spin()
