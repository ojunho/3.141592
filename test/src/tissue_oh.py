#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
import time

from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from obstacle_detector.msg import Obstacles

from SlideWindow import SlideWindow
from Warper import Warper
from lidar_mode import lidar_mode


bridge = CvBridge()
warper = Warper()
slideWindow = SlideWindow()

cv_image = np.empty(shape=[0])
motor = None
mode = None
initialized = False
last_x_location = 320
x_loc = 0
flag = False
last_ob_mode = None
ob_dir = None

CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("mouse pos (x, y)", x, y)

def nothing(x):
    pass

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()

    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)

def decision_Obstacles(data):
    global mode
    left_ob = []
    right_ob = []
    for i in data.circles:
        if -0.4 < i.center.x < 0:
            if -0.3 < i.center.y < 0:
                left_ob.append(i)
            elif 0 < i.center.y < 0.3:
                right_ob.append(i)
    
    if len(left_ob) > 0: #왼쪽 장애물
        left_ob_close = left_ob[0]
        if(left_ob_close.center.y + left_ob_close.radius < 0.2):
            mode = "right_drive"
        else:
            mode = "left_drive"
    elif len(right_ob) > 0: #오른쪽 장애물
        right_ob_close = right_ob[0]
        if(right_ob_close.center.y - right_ob_close.radius > 0.2):
            mode = "left_drive"
        else:
            mode = "right_drive"
    else:
        mode = "lane_drive"

# 실질적인 메인 함수.
def start():
    global motor, cv_image, initialized, flag, last_x_location, x_loc, mode

    rospy.init_node('my_driver')

    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

    rospy.Subscriber('raw_obstacles', Obstacles, decision_Obstacles)

    print ("----- Xycar self driving -----")
    rospy.wait_for_message("/usb_cam/image_raw/", Image)

    while not rospy.is_shutdown():
        img = cv_image.copy()
        mid = 320
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)

        low_threshold = 70
        high_threshold = 210
        canny = cv2.Canny(blur_gray, low_threshold, high_threshold)
        kernel = np.ones((5, 5), np.uint8)
        close = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)
        warp_img = warper.warp(close)
        ret, thres_img = cv2.threshold(warp_img, 150, 255, cv2.THRESH_BINARY)
        # mode, count = check_stopline(thres_img)
        # if(mode == "StopLine"):
        #     time.sleep(3.5)

        cv2.imshow("processed_img", thres_img)
        cv2.waitKey(1)

        slide_img, x_location, _ = slideWindow.slidewindow(thres_img,  flag)
        cv2.setMouseCallback("processed_img", mouse_callback)
        x_loc = x_location

        # print(mode)
        print(mode)

        if x_location == None:
            flag = True
            x_location = last_x_location
        
        else:
            last_x_location = x_location
            # if flag == True and x_location > 279:
            #     x_location += 13
            flag = False

        cv2.imshow("window_view", slide_img)
        cv2.waitKey(1)

        if(mode == "left_drive"): 
            print("left")
            mid = 380
        elif(mode == "right_drive"):
            print("right") 
            mid = 260
        else:
            mid = 320

        angle = int((mid - int(x_location))*-1)

        # print(mode)
        

        # if(mode == "left_drive"): 
        #     angle = -50
        # elif(mode == "right_drive"): 
        #     angle = 50
        # else:
        #     mid = 320
        #     angle = int((mid - int(x_location))*-1)


        print(angle)
        
        # speed = max(5, 40 - abs(angle))
        speed = 4
        drive(angle, speed)

if __name__ == '__main__':
    start()
