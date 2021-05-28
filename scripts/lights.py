#!/usr/bin/env python
import rospy

from assisted_cleaning_solution.msg import Task

class task_subscriber:

    def __init__(self):
        rospy.Subscriber('task', Task, self.lights_callback, queue_size=1)

    def lights_callback(self, msg):
        print('Color light = ', msg.task)

def main():
    rospy.init_node('lights')
    task_subscriber()
    rospy.spin()

if __name__ == '__main__':
    main()