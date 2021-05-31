#!/usr/bin/env python
import rospy

from smach import State

from assisted_cleaning_solution.msg import Task, CleanPlinthsActionResult
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

# ---------------------------------------------------------------------
class receive_task(State):
    # Read out the message task and then decide what to do next
    def __init__(self):
        State.__init__(self, 
            outcomes = ['nextRoom', 'plinths', 'chairs', 'not_received', 'preempted'],
            input_keys = ['previous_task'],
            output_keys = ['new_task', 'previous_task'])

        rospy.Subscriber('/task', Task, self.receive_task_cb)
        self.task_num = None
    
    def receive_task_cb(self, task):
        self.task_num = task.task

    def execute(self, userdata):
        if (self.preempt_requested()):
            userdata.previous_task = 0
            self.service_preempt()
            return 'preempted'

        if (self.task_num != userdata.previous_task):
            userdata.new_task = self.task_num
            userdata.previous_task = self.task_num

            if (self.task_num == -2):
                return 'plinths'

            if (self.task_num == -3):
                return 'chairs'

            if (self.task_num > 2):
                return 'nextRoom'

        return 'not_received'


# ---------------------------------------------------------------------
# Cancel the move base goal. 
class cancel_goal(State):
    def  __init__(self):
        State.__init__(self, 
            outcomes = ['cancel_outcome'])
        self.cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size = 10)

    def execute(self, userdata):
        goalId = GoalID()
        self.cancel_pub.publish(goalId)    
        return 'cancel_outcome'

# ---------------------------------------------------------------------
# Be able to press button for continue before or while removing object.
class path_clear(State):
    def __init__(self):
        State.__init__(self, 
            outcomes = ['clear', 'not_clear', 'time_up', 'move_around', 'preempted'])
        rospy.Subscriber('/ir_front', Range, self.path_clear_cb)

        self.range = 0
        self.start = 0
        self.time_start = 0
        self.count = 0

    def path_clear_cb(self, ir_data):
        self.range = ir_data.range

    def execute(self,userdata):
        time_now = rospy.get_time()

        if (self.start == 0):
            self.time_start = rospy.get_time()
            self.start = 1

        if (self.preempt_requested()):
            self.start = 0
            self.count = 0
            self.service_preempt()
            return 'preempted'

        if (self.range > 0.15):
            self.start = 0
            self.count = 0
            return 'clear'

        if (time_now - self.time_start > 3):
            self.start = 0

            if (self.count > 0):
                self.count = 0
                return 'move_around'

            self.count = self.count + 1
            return 'time_up'

        else:
            return 'not_clear'

# ---------------------------------------------------------------------
# The move function to control the robot
def move(x, yaw):
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    vel_msg.linear.x = x
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = yaw

    velocity_publisher.publish(vel_msg)