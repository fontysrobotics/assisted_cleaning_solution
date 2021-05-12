#!/usr/bin/env python
import rospy

import rospy
from smach import cb_interface

from assisted_cleaning_solution.msg import Task, FindChairsResult
from assisted_cleaning_solution.srv import SelectCoordsRequest
from move_base_msgs.msg import MoveBaseGoal

# ---------------------------------------------------------------------
# Monitor if the message is stop. 
# Also used to start the program when state is RESET 
def monitor(userdata, msg):
    if (msg.task == 0):
        return False
    return True

# ---------------------------------------------------------------------
# Update previous task when preempted 
@cb_interface(outcomes=['preempted'], output_keys=['previous_task'])
def update_task(userdata):
    userdata.previous_task = 0
    return 'preempted'

# ---------------------------------------------------------------------
# Update Task message that robot is done 
@cb_interface(outcomes=['succeeded'], output_keys=['previous_task'])
def task_completed(userdata):
    userdata.previous_task = 0

    pub_task = rospy.Publisher('/task', Task, queue_size = 10)
    message = Task()
    message.task = 0
    pub_task.publish(message)
    return 'succeeded'

# ---------------------------------------------------------------------
# data for the select coords
@cb_interface(input_keys = ['room', 'coords'])
def select_room(userdata, request):
    select_room_request = SelectCoordsRequest()

    select_room_request.file = 'RoomCoords'
    select_room_request.row = userdata.room - 3
    select_room_request.column = 1
    select_room_request.coords = userdata.coords

    return select_room_request 

# ---------------------------------------------------------------------
# Decide if follow plinths is ended because of obstacle or because of succeeded
@cb_interface(outcomes=['obstacle'])
def result_done_or_obstacle(userdata, status, result):
    if (result.done == False):
        return 'succeeded'
        
    if (result.done == True):
        return 'obstacle'

# ---------------------------------------------------------------------
# Decide if response means the robot needs to make a turn or continue
@cb_interface(outcomes=['turn','continue'])
def result_obstacle(userdata, status, result):
    if (result == None):
        return 'continue'

    elif (result.reaction == True):
        return 'turn'

    elif (result.reaction == False):
        return 'continue'

# ---------------------------------------------------------------------
# Give the move_base a goal
def goal_planner(userdata, goal):
    goal_planner = MoveBaseGoal()

    goal_planner.target_pose.header.frame_id = 'map'
    goal_planner.target_pose.pose.position.x = userdata.array[0]
    goal_planner.target_pose.pose.position.y = userdata.array[1]
    goal_planner.target_pose.pose.orientation.z = userdata.array[2]
    goal_planner.target_pose.pose.orientation.w = userdata.array[3]

    return goal_planner

# ---------------------------------------------------------------------
# The result of the navigation of the move_base
def result_planner(userdata, status, result):
    if status == 3:
        return 'succeeded'
    if status == 2:
        return 'preempted'
    else:
        return 'aborted'

# ---------------------------------------------------------------------
# data for the select coords
@cb_interface(input_keys = ['count_steps', 'room_number'])
def select_path(userdata, request):
    select_room_request = SelectCoordsRequest()

    select_room_request.file = 'PathCoords'
    select_room_request.row = userdata.count_steps
    select_room_request.column = userdata.room_number # Correct info
    # select_room_request.column = 1 # Easier to test with
    select_room_request.coords = [0,0,0,0]

    return select_room_request

# ---------------------------------------------------------------------
# Moving to the check nearby for chairs when chairs gets interupted
@cb_interface(outcomes = ['done_finding_chair'])
def response_next_state(userdata, response):
    if (response.count > 7):
        return 'done_finding_chair'
    else:
        return 'succeeded'

# ---------------------------------------------------------------------
# The result when the robot is following the path around the chairs
@cb_interface(outcomes = ['done_finding_chair'], input_keys = ['count_steps'])
def result_find_chairs(userdata, status, result):
    if FindChairsResult:
        if userdata.count_steps == 7:
            return 'done_finding_chair'
        else:
            return 'succeeded'

# ---------------------------------------------------------------------
# The response of check if a chair is nearby
@cb_interface(outcomes=['none_nearby','nearby'], input_keys = ['count_steps'], output_keys = ['count_steps'])
def response_check_chairs(userdata, response):
    if (response.next_state == 0):
        userdata.count_steps = 0
        return 'succeeded'
    if (response.next_state == 1):
        return 'none_nearby'
    if (response.next_state == 2):
        return 'nearby'
 
# ---------------------------------------------------------------------
# The response when the robot is following the path back around the chairs
@cb_interface(outcomes=['end_of_path'], output_keys = ['count', 'coords'])
def response_pose_chair(userdata, response):
    if (response.count == 10):
        userdata.count = 0
        return 'end_of_path'
    else:
        userdata.coords = response.coords
        userdata.count = response.count
        return 'succeeded'

# ---------------------------------------------------------------------
# Infill to test transition
@cb_interface(outcomes=['succeeded','done_cleaning'], input_keys = ['times_cleaned'], output_keys = ['times_cleaned'])
def count_clean_chair(userdata):
    print('times_cleaned', userdata.times_cleaned)
    if userdata.times_cleaned == 2:
        userdata.times_cleaned = 0
        return 'done_cleaning'
    else:
        userdata.times_cleaned = userdata.times_cleaned + 1
        return 'succeeded'

# ---------------------------------------------------------------------
# Remove the chair from the list 
@cb_interface(outcomes=['succeeded'], input_keys = ['x_coord_chair', 'y_coord_chair', 'x_coords_chair','y_coords_chair'],output_keys = ['x_coords_chair','y_coords_chair'])
def remove_chair_from_list(userdata):
    rounded_x_coords = [round(num, 8)for num in list(userdata.x_coords_chair)]
    rounded_y_coords = [round(num, 8)for num in list(userdata.y_coords_chair)]
    
    rounded_x_coords.remove(round(userdata.x_coord_chair, 8))
    rounded_y_coords.remove(round(userdata.y_coord_chair, 8))

    userdata.x_coords_chair = rounded_x_coords
    userdata.y_coords_chair = rounded_y_coords

    return 'succeeded'