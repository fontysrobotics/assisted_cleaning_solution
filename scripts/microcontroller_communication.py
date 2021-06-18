#!/usr/bin/env python
import rospy
import serial

from assisted_cleaning_solution.msg import Task

class task_subscriber:

    def __init__(self):
        self.microcontroller = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.01)

        rospy.Subscriber('task', Task, self.lights_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def write(self, x):
        arrayByte = str(x) + '\n\r'
        self.microcontroller.write(arrayByte.encode())

    def lights_callback(self, msg):
        self.write(msg.task)

    def timer_callback(self, timer):
        try:
            data = self.microcontroller.readline()
            print(data.decode())
            self.microcontroller.flushInput()

        except KeyboardInterrupt:
            self.microcontroller.close()

def main():
    rospy.init_node('microcontroller_communication')
    task_subscriber()
    rospy.spin()

if __name__ == '__main__':
    main()