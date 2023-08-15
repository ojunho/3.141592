#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math
from time import sleep
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Int32, Float32
from xycar_msgs.msg import xycar_motor

motor = None

class Rabacon_drive:
    
    def __init__(self):
        rospy.Subscriber("raw_obstacles", Obstacles, self.Rabacon)
        self.angle = 0.0  # Initialize angle as a class attribute

    def Rabacon(self, data):
        left_rabacon = []
        right_rabacon = []
        print("go")
        for i in data.circles:
            if i.center.x > -1.5 and i.center.x < -0.05:
                if i.center.y > 0 and i.center.y < 0.7:
                    left_rabacon.append(i)
                elif i.center.y < 0 and i.center.y > -0.7:
                    right_rabacon.append(i)
        
        if len(left_rabacon) > 0 and len(right_rabacon) > 0:
            left_close_rabacon = sorted(left_rabacon, key = lambda x : -x.center.x)[0]
            right_close_rabacon = sorted(right_rabacon, key = lambda x : -x.center.x)[0]
            avg_x_rabacon = (left_close_rabacon.center.x + right_close_rabacon.center.x) / 2
            avg_y_rabacon = (left_close_rabacon.center.y + right_close_rabacon.center.y) 
            self.angle = avg_y_rabacon * 150 # Store angle as a class attribute
            print("rabacon_drive")
            print(self.angle)
        else:
            print("no_rabacon")

def drive(angle, speed):
    global motor
    motor_msg = xycar_motor()
    motor_msg.angle = angle 
    motor_msg.speed = speed
    motor.publish(motor_msg)
    
def start():
    rospy.init_node('my_driver')
    rabacon = Rabacon_drive()  # Create an instance of Rabacon_drive class

    global motor
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    while not rospy.is_shutdown():
        # Use the angle value from the Rabacon_drive instance
        angle_to_drive = int(rabacon.angle)
        speed = max(8, 15 - abs(angle_to_drive)/3)
        drive(angle_to_drive, speed)  # Example: Drive with a speed of 0.5
        sleep(0.1)  # Example: Sleep for 0.1 seconds to control the update rate

if __name__ == '__main__':
    start()
