#!/usr/bin/env python

import rospy
import numpy
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt

from assisted_cleaning_solution.srv import GroupChairs, GroupChairsResponse

def group_points(x_coords_chair, y_coords_chair):
    # Grouping the dots together
    x = numpy.array(x_coords_chair)
    y = numpy.array(y_coords_chair)
    coords_chair = numpy.column_stack((x,y))
    plt.scatter(x_coords_chair, y_coords_chair)

    tree = cKDTree(coords_chair)
    rows_to_fuse = tree.query_pairs(r = 0.6, output_type = 'ndarray')

    # While loop will continue till no points are close together anymore
    # Two dots at the time will be merged
    while (len(rows_to_fuse) > 0):
        fused = numpy.mean(coords_chair[rows_to_fuse[0]], axis = 0)

        coords_chair = numpy.delete(coords_chair, rows_to_fuse[0], 0)
        coords_chair = numpy.append(coords_chair , [fused], 0)

        tree = cKDTree(coords_chair)
        rows_to_fuse = tree.query_pairs(r = 0.6, output_type = 'ndarray')

    plt.scatter(coords_chair[:,0],coords_chair[:,1], color = 'black')
    plt.show()
    return coords_chair[:,0], coords_chair[:,1]

def handle_group_chairs(req):
    _result = GroupChairsResponse()
    x, y = group_points(req.x_coords_chair, req.y_coords_chair)

    _result.x_coords_chair_group = x.tolist()
    _result.y_coords_chair_group = y.tolist()
    return _result

if __name__ == "__main__":
    rospy.init_node('group_chairs')
    service = rospy.Service('group_chairs_service', GroupChairs, handle_group_chairs)
    rospy.spin()