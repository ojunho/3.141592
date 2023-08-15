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

class lidar_mode:
    
    def __init__(self):
        rospy.Subscriber("raw_obstacles", Obstacles, self.Rabacon)
        self.angle = 0.0  # Initialize angle as a class attribute

    def Rabacon(self, data):
        left_rabacon = []
        right_rabacon = []
        for i in data.circles:
            if i.center.x > -1.5 and i.center.x < 0:
                if i.center.y > 0 and i.center.y < 1.5:
                    left_rabacon.append(i)
                elif i.center.y < 0 and i.center.y > -1.5:
                    right_rabacon.append(i)

        if len(left_rabacon) > 0 and len(right_rabacon) > 0:
            left_close_rabacon = sorted(left_rabacon, key = lambda x : -x.center.x)[0]
            right_close_rabacon = sorted(right_rabacon, key = lambda x : -x.center.x)[0]
            avg_x_rabacon = (left_close_rabacon.center.x + right_close_rabacon.center.x) / 2
            avg_y_rabacon = (left_close_rabacon.center.y + right_close_rabacon.center.y) / 2
            self.angle = math.atan2(avg_y_rabacon, avg_x_rabacon) * 15 # Store angle as a class attribute
            return "rabacon_drive"
        elif len(left_rabacon) > 0:
            return "left_drive"
        elif len(right_rabacon) > 0:
            return "right_drive"
        else:
            return "lane_drive"
