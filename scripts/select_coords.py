#!/usr/bin/env python
import rospy
import csv
import rospkg
from itertools import islice

from assisted_cleaning_solution.srv import SelectCoords, SelectCoordsResponse


def select_coords(file_name, row, column):
    # Read CSV file and turn the value in list
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('assisted_cleaning_solution')
    
    with open('%s/%s.csv' % (package_path, file_name), 'r') as file:
        reader = csv.reader(file, delimiter = ';')

        coords = next(islice(reader, row + 1, None))[column]
        coords = coords.split(',')
        coords = [float(item) for item in coords]

    return coords

def handle_select_coords(req):
    _response = SelectCoordsResponse()
    coords = select_coords(req.file, req.row, req.column)

    if (coords == [1000,1000,1000,1000]): # To go to default position in room
        coords = req.coords

    elif (req.file == 'RoomCoords'): # The number of the room so the chairs has the correct column
        _response.count = req.row - 1

    else: # For the next time the next row will be chosen
        _response.count = req.row + 1

    _response.coords = coords

    return _response

if __name__ == "__main__":
    rospy.init_node('select_coords')
    service = rospy.Service('select_coords_service', SelectCoords, handle_select_coords)
    rospy.spin()




