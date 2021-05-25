#!/usr/bin/env python
import rospy

from assisted_cleaning_solution.msg import Task

def lights_callback(msg):
    print('Color light = ',msg.task)
    
def task_listener():
    rospy.init_node('lights', anonymous=True)
    rospy.Subscriber('task', Task, lights_callback)
    rospy.spin()

if __name__ == '__main__':
    task_listener()