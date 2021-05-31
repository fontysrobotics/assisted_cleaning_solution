#!/usr/bin/env python
import rospy
import math

from nav_msgs.msg import Odometry
from assisted_cleaning_solution.srv import CheckChairs, CheckChairsResponse 

class checkChairs:
    def __init__(self):
        self.x_odom = None
        self.y_odom = None

    def subs_chair(self):
        rospy.Subscriber('odom', Odometry, self.odomCB)
        service = rospy.Service('check_chairs_service', CheckChairs, self.handle_check_chairs)

    def odomCB(self, message):
        self.x_odom = message.pose.pose.position.x
        self.y_odom = message.pose.pose.position.y

    def select_next_step(self, x_coords_chairs, y_coords_chairs):
        # When no chairs are left the Statemachine chairs will be done.
        if (len(x_coords_chairs) == 0):
            return 0, 0, 0

        # Else there will be decide if there are chairs nearby to go to
        else:
            for index in range(len(x_coords_chairs)):
                distance_to_chair = math.hypot(x_coords_chairs[index]-self.x_odom, y_coords_chairs[index]-self.y_odom)

                # When chairs are nearby
                if (distance_to_chair < 1.5):
                    return 2, x_coords_chairs[index], y_coords_chairs[index]

            # When no chairs are nearby
            return 1, 0, 0

    def handle_check_chairs(self, req):
        _response = CheckChairsResponse()

        next_state, x_coord, y_coord = self.select_next_step(req.x_coords_chair, req.y_coords_chair)

        _response.next_state = next_state
        _response.x_coord_chair = x_coord
        _response.y_coord_chair = y_coord
        return _response

if __name__ == "__main__":
    rospy.init_node('check_chairs_nearby')
    chairs = checkChairs()
    chairs.subs_chair()
    rospy.spin()