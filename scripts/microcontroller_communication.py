#!/usr/bin/env python
from os import write
import rospy
import serial

from geometry_msgs.msg import Twist
from assisted_cleaning_solution.msg import Task, Sensor

class task_subscriber:

    def __init__(self):
        self.microcontroller = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.001)
        self.sensor_publisher = rospy.Publisher('sensor', Sensor, queue_size=1)

        self.task_subsriber = rospy.Subscriber('task', Task, self.lights_callback, queue_size=1)
        self.twist_subscriber = rospy.Subscriber('cmd_vel', Twist, self.twist_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def publish_sensor(self, data):
        sensor = Sensor()
       
        sensor.ir_back = data[0]
        sensor.ir_front = data[1]
        sensor.sonar1 = data[2]
        sensor.sonar2 = data[3]

        self.sensor_publisher.publish(sensor)

    def lights_callback(self, msg):
        self.write(msg.task)

    def twist_callback(self, vel_msg):
        x = vel_msg.linear.x
        z = vel_msg.angular.z
        radius = 0.11
        left_vel = x - (z*radius)/2
        right_vel = x + (z*radius)/2

        if left_vel < 0:
            left_dir = 1
        else:
            left_dir = 0
        if right_vel < 0:
            right_dir = 1
        else:
            right_dir = 0

        left_dutycycle = 65535 * abs(left_vel)/0.3
        right_dutycycle = 65535 * abs(right_vel)/0.3

        left_dutycycle = int(left_dutycycle)
        right_dutycycle = int(right_dutycycle)

        if left_dutycycle > 65535:
            left_dutycycle = 65535
        if right_dutycycle > 65535:
            right_dutycycle = 65535
            
        self.write(['motor', left_dutycycle, left_dir, right_dutycycle, right_dir])

    def timer_callback(self, timer):
        try:
            data = self.microcontroller.readline()
            if 'send' in data.decode():
                data = (data.decode()).split()
                data.pop(0)
                data = [float(i) for i in data]

                self.microcontroller.flushInput()

                if len(data) == 4:
                    self.publish_sensor(data)
            else:
                pass
            

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
