#!/usr/bin/env python
import rospy
import serial

from assisted_cleaning_solution.msg import Task, Sensor

class task_subscriber:

    def __init__(self):
        self.microcontroller = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.01)
        self.sensor_publisher = rospy.Publisher('sensor', Sensor, queue_size=10)

        self.task_subsriber = rospy.Subscriber('task', Task, self.lights_callback, queue_size=1)
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