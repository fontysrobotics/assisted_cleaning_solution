#!/usr/bin/env python

import roslib; roslib.load_manifest('assisted_cleaning_solution')

import rospy
import actionlib

import math
import numpy as np   
import matplotlib.pyplot as plt
import tf
import tf2_ros
from scipy.spatial import cKDTree

from sensor_msgs.msg import LaserScan

from assisted_cleaning_solution.msg import FindChairsAction, FindChairsResult

class findChairs:
    _result = FindChairsResult()

    def __init__(self):
        self.goal_received = False

        self.odomX = None
        self.odomY = None
        self.odomYaw = None
        self.scan_range_list = None

        self.x_coords_chair = []
        self.y_coords_chair = []

    def subs_chairs(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.server_chairs = actionlib.SimpleActionServer('find_chairs_action', FindChairsAction, 
            auto_start = False)

        self.server_chairs.register_goal_callback(self.goalCB)
        self.server_chairs.register_preempt_callback(self.preemptCB)
        
        self.server_chairs.start()
   
        sub_scan = rospy.Subscriber('scan2', LaserScan, self.laserCB)


    def goalCB(self):
        self.rounded_scan_range_list = []
        self.x_list = []
        self.y_list = []
        self.range_average_list = []
        self.slope_list = []
        self.intercept_list = []
        self.theta_list = []
        self.index_correct_angles = []
        self.index_chairs = []

        trans = self.tfBuffer.lookup_transform('map', 'lidar_2', rospy.Time())
        euler = tf.transformations.euler_from_quaternion([0, 0, trans.transform.rotation.z, trans.transform.rotation.w])

        self.odomX = trans.transform.translation.x
        self.odomY = trans.transform.translation.y
        self.odomYaw = euler[2]

        self.goal_received = self.server_chairs.accept_new_goal()
        self.goal_received = True

    def preemptCB(self):
        self.goal_received = False
        self.server_chairs.set_preempted()

    def laserCB(self, LaserScan):
        self.scan_range_list = LaserScan.ranges
    
        self.scan_range_list = self.scan_range_list[0::1]
        self.angle_between = LaserScan.angle_increment * 1

        if (self.goal_received == True):
            self.mag_dir_to_component()
            self.approximate_line_points()
            self.angle_between_slopes()
            self.check_location_points()

            # self.plot(4)

            if(self.plotted == True):
                self.plotted = False
                self.goal_received = False
                self._result.x_coords_chair = self.x_coords_chair
                self._result.y_coords_chair = self.y_coords_chair
                self.server_chairs.set_succeeded(self._result)


    def mag_dir_to_component(self):
        # Change te data from magnitute en direction coordinates to x and y components
        for index in range(len(self.scan_range_list)):
            x = self.scan_range_list[index] * math.cos(-math.pi + (self.angle_between * index))
            y = self.scan_range_list[index] * math.sin(-math.pi + (self.angle_between * index))

            self.x_list.append(x)
            self.y_list.append(y)

    def approximate_line_points(self):
        # Calculate the slope of 20 points to aprroximate the lines.
        # For more precise point on map there should be checked. (important for driving around the chair)
        # If the twenty points used to calculate the line are close enough together.
        # The lines will be drawn through half a chair and half the wall.
        for index in range(0, len(self.x_list), 5):
            approx_slope, approx_intersect = np.polyfit(self.x_list[index:index+15], self.y_list[index:index+15], 1)
            
            self.slope_list.append(approx_slope)
            self.intercept_list.append(approx_intersect)

    def angle_between_slopes(self):
        # Calculate the angle between two slopes next to each other in the list
        for index in range(len(self.slope_list)-1):

            theta = math.atan2((self.slope_list[index] - self.slope_list[index+1]), (1 + self.slope_list[index] * self.slope_list[index+1]))
            self.theta_list.append(theta)

            # The points with an angle close to the angle of the chair are added to a list
            if ((theta > 1.1 and theta < 1.5) or (theta > -1.5 and theta < -1.1)):
                self.index_correct_angles.append(index)

    def check_location_points(self):
        # check if the points are in the room
        for index in self.index_correct_angles:
            # Decide the coordinates of the points
            x = (self.intercept_list[index] - self.intercept_list[index+1]) / (self.slope_list[index+1] - self.slope_list[index])
            y = self.intercept_list[index] + self.slope_list[index] * x

            # Decide what way the robot and lidar are pointing.
            if (self.odomYaw > -1.7 and self.odomYaw < -1.44):
                x_point_map = self.odomX + x
                y_point_map = self.odomY + y

                if not (x_point_map > 5.4 or x_point_map < 0.53 or y_point_map > 6.5 or y_point_map < 1.5):
                    self.index_chairs.append(index)
                    self.x_coords_chair.append(x_point_map)
                    self.y_coords_chair.append(y_point_map)
                    
            if (self.odomYaw > -0.3 and self.odomYaw < 0.3):
                x_point_map = self.odomX - y
                y_point_map = self.odomY + x

                if not (x_point_map > 5.4 or x_point_map < 0.53 or y_point_map > 6.5 or y_point_map < 1.5):
                    self.index_chairs.append(index)
                    self.x_coords_chair.append(x_point_map)
                    self.y_coords_chair.append(y_point_map)

            if (self.odomYaw > 1.44 and self.odomYaw < 1.7):
                x_point_map = self.odomX - x
                y_point_map = self.odomY - y

                if not (x_point_map > 5.4 or x_point_map < 0.53 or y_point_map > 6.5 or y_point_map < 1.5):
                    self.index_chairs.append(index)
                    self.x_coords_chair.append(x_point_map)
                    self.y_coords_chair.append(y_point_map)

            if (self.odomYaw > 2.84 and self.odomYaw < 3.44):
                x_point_map = self.odomX + y
                y_point_map = self.odomY - x

                if not (x_point_map > 5.4 or x_point_map < 0.53 or y_point_map > 6.5 or y_point_map < 1.5):
                    self.index_chairs.append(index)
                    self.x_coords_chair.append(x_point_map)
                    self.y_coords_chair.append(y_point_map)

        self.plotted = True
        rospy.sleep(1)


    def plot(self,choice):
        # This is so the points can be shown in a plot.
        print('plot')
        fig, ax = plt.subplots()
        plt.scatter(self.x_list, self.y_list, 2)

        x_vals = np.array(plt.xlim([-4,8]))

        if (choice == 0):
            for index in range(len(self.slope_list)):
                y_vals = self.intercept_list[index] + self.slope_list[index] * x_vals
                plt.plot(x_vals, y_vals, '--')

        if (choice == 1):
            for index in self.index_correct_angles:
                x = (self.intercept_list[index] - self.intercept_list[index+1]) / (self.slope_list[index+1] - self.slope_list[index])
                y = self.intercept_list[index] + self.slope_list[index] * x
                plt.scatter(x,y,20)

                y_vals = self.intercept_list[index] + self.slope_list[index] * x_vals
                plt.plot(x_vals, y_vals, '--')
                y_vals = self.intercept_list[index+1] + self.slope_list[index+1] * x_vals
                plt.plot(x_vals, y_vals, '--')

        if (choice == 2):
            for index in self.index_correct_angles:
                y_vals = self.intercept_list[index] + self.slope_list[index] * x_vals
                plt.plot(x_vals, y_vals, '--')
                y_vals = self.intercept_list[index+1] + self.slope_list[index+1] * x_vals
                plt.plot(x_vals, y_vals, '--')

        if (choice == 3):
            for index in self.index_chairs:
                x = (self.intercept_list[index] - self.intercept_list[index+1]) / (self.slope_list[index+1] - self.slope_list[index])
                y = self.intercept_list[index] + self.slope_list[index] * x
                plt.scatter(x,y,20)

                y_vals = self.intercept_list[index] + self.slope_list[index] * x_vals
                plt.plot(x_vals, y_vals, '--')
                y_vals = self.intercept_list[index+1] + self.slope_list[index+1] * x_vals
                plt.plot(x_vals, y_vals, '--')

        if (choice == 4):
            for index in range(len(self.x_coords_chair)):
                plt.scatter(self.x_coords_chair[index], self.y_coords_chair[index], color = 'black')

            for index in self.index_chairs:
                x = (self.intercept_list[index] - self.intercept_list[index+1]) / (self.slope_list[index+1] - self.slope_list[index])
                y = self.intercept_list[index] + self.slope_list[index] * x
                plt.scatter(x,y,20)
            

        plt.xlabel('x-as')
        plt.ylabel('y-as')
        plt.ylim([-5,6])
        plt.grid()
        self.plotted = True
        plt.show()
        rospy.sleep(1)
        

if __name__ == "__main__":
    rospy.init_node('find_chairs')
    chairs = findChairs()
    chairs.subs_chairs()
    rospy.spin()