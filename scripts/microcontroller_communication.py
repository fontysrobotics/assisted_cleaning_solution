#!/usr/bin/env python
from os import write
import rospy
import serial

from geometry_msgs.msg import Twist
from assisted_cleaning_solution.msg import Task, Sensor

class task_subscriber:

    def __init__(self):
        self.microcontroller = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.01)
        self.sensor_publisher = rospy.Publisher('sensor', Sensor, queue_size=10)

        self.task_subsriber = rospy.Subscriber('task', Task, self.lights_callback, queue_size=1)
        self.twist_subscriber = rospy.Subscriber('cmd_vel', Twist, self.twist_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def publish_sensor(self, ir_back, ir_front, sonar1, sonar2):
        sensor = Sensor()

        sensor.ir_back = ir_back
        sensor.ir_front = ir_front
        sensor.sonar1 = sonar1
        sensor.sonar2 = sonar2

        self.sensor_publisher.publish(sensor)

    def lights_callback(self, msg):
        self.write(msg.task)

    def twist_callback(self, vel_msg):
        x = vel_msg.linear.x
        z = vel_msg.angular.z
        radius = 0.11
        left_vel = x - (z*radius)/2
        right_vel = x + (z*radius)/2

        left_dutycycle = 100 * left_vel
        right_dutycycle = 100 * right_vel

        self.write((left_dutycycle,right_dutycycle))

    def timer_callback(self, timer):
        try:
            data = self.microcontroller.readline()
            print(data.decode())
            self.microcontroller.flushInput()

            self.publish_sensor(1,2,3,4)

        except KeyboardInterrupt:
            self.microcontroller.close()

    def write(self, x):
        arrayByte = str(x) + '\n\r'
        self.microcontroller.write(arrayByte.encode())
            
def main():
    rospy.init_node('microcontroller_communication')
    task_subscriber()
    rospy.spin()

if __name__ == '__main__':
    main()