#!/usr/bin/env python
import rospy
import serial
import struct

from assisted_cleaning_solution.msg import Task

class task_subscriber:

    def __init__(self):
        self.microcontroller = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.1)
        rospy.Subscriber('task', Task, self.lights_callback, queue_size=1)

    def write_read(self, x):
        arrayByte = bytearray(struct.pack('f', x))
        self.microcontroller.write(arrayByte)
        
        print('Color light = ', x, arrayByte, struct.unpack('f', arrayByte))

    def lights_callback(self, msg):
        self.write_read(msg.task)
    
def main():
    rospy.init_node('lights')
    task_subscriber()
    rospy.spin()

if __name__ == '__main__':
    main()