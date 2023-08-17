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
last_ob_mode = "No"
ob_dir = None
ob_cal = 0

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


# def check_stopline(image):
#     area = image[240:380, 185:380]
#     stopline_count = cv2.countNonZero(area)
#     # print(stopline_count)
#     if stopline_count > 1650:
#         return "StopLine", stopline_count
#     else:
#         return "no"

def decision_Obstacles(data):
    global mode
    left_ob = []
    right_ob = []
    for i in data.circles:
        if i.center.x > -0.4 and i.center.x < 0:
            if i.center.y < 0 and i.center.y > -0.4:
                left_ob.append(i)
            elif i.center.y > 0 and i.center.y < 0.4:
                right_ob.append(i)
    
    if len(left_ob) > 0: #왼쪽 장애물
        left_ob_close = left_ob[0]
        if(abs(left_ob_close.center.y + left_ob_close.true_radius) > 0.35):
            mode = "left_drive"
        elif(abs(left_ob_close.center.y + left_ob_close.true_radius) > 0.45):
            mode = "right_drive"
    elif len(right_ob) > 0: #오른쪽 장애물
        right_ob_close = right_ob[0]
        if(abs(right_ob_close.center.y + right_ob_close.true_radius) > 0.35):
            mode = "right_drive"
        elif(abs(right_ob_close.center.y + right_ob_close.true_radius) < 0.45):
            mode = "left_drive"
        else:
            mode = "lane_drive"
    else:
        mode = "lane_drive"

def decision_Obstacles2(data):
    global mode
    left_ob = []
    right_ob = []
    for i in data.circles:
        if i.center.x > -0.4 and i.center.x < 0.0:
            if i.center.y < 0 and i.center.y > -0.3:
                left_ob.append(i)
            elif i.center.y > 0 and i.center.y < 0.3:
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


def decision_Obstacles3(data):
    global mode, last_ob_mode, ob_dir
    left_ob = []
    right_ob = []
    for i in data.circles:
        if i.center.x > -0.6 and i.center.x < -0.1:
            if i.center.y < 0 and i.center.y > -0.4:
                left_ob.append(i)
            elif i.center.y > 0 and i.center.y < 0.4:
                right_ob.append(i)
    
    if len(left_ob) > 0: #왼쪽 장애물
        # print("left")
        ob_dir = "left_ob"
        left_ob_close = left_ob[0]
        if(last_ob_mode == "left_ob"):
            mode = "left_drive"
        elif(left_ob_close.center.y + left_ob_close.radius < 0.2):
            mode = "right_drive"
        else:
            mode = "left_drive"
    elif len(right_ob) > 0: #오른쪽 장애물
        # print("right")
        ob_dir = "right_ob"
        right_ob_close = right_ob[0]
        if(last_ob_mode == "right_ob"):
            mode = "right_drive"
        elif(right_ob_close.center.y - right_ob_close.radius > -0.2):
            mode = "left_drive"
        else:
            mode = "right_drive"
    else:
        last_ob_mode = ob_dir
        mode = "lane_drive"

    # print(f"LEFT: {len(left_ob)}\nRIGHT: {len(right_ob)}")
    # print(last_ob_mode)
    print(mode)

def decision_Obstacles5(data):
    global mode, last_ob_mode, ob_dir, ob_cal
    left_ob = []
    right_ob = []
    for i in data.circles:
        if i.center.x > -0.6 and i.center.x < -0.1:
            if i.center.y < 0 and i.center.y > -0.4:
                left_ob.append(i)
            elif i.center.y > 0 and i.center.y < 0.4:
                right_ob.append(i)
    
    if len(left_ob) > 0: #왼쪽 장애물
        # print("left")
        ob_dir = "left_ob"
        left_ob_close = left_ob[0]
        if(last_ob_mode == "left_ob"):
            mode = "left_drive"
        elif(left_ob_close.center.y + left_ob_close.radius < 0.2):
            mode = "right_drive"
        else:
            mode = "left_drive"
    elif len(right_ob) > 0: #오른쪽 장애물
        # print("right")
        ob_dir = "right_ob"
        right_ob_close = right_ob[0]
        angle = right_ob_close.center.y
        if(last_ob_mode == "right_ob"):
            mode = "right_drive"
        elif(right_ob_close.center.y - right_ob_close.radius > -0.2):
            mode = "left_drive"
        else:
            mode = "right_drive"
    else:
        if(ob_dir != last_ob_mode):
            ob_cal += 1
        last_ob_mode = ob_dir

    # print(f"LEFT: {len(left_ob)}\nRIGHT: {len(right_ob)}")
    # print(last_ob_mode)
    print(mode)


#test angle


# 실질적인 메인 함수.
def start():
    global motor, cv_image, initialized, flag, last_x_location, x_loc, mode

    rospy.init_node('my_driver')

    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

    rospy.Subscriber('raw_obstacles', Obstacles, decision_Obstacles3)

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
        # print(last_ob_mode)

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
            # print("left")
            angle = -35
        elif(mode == "right_drive"):
            # print("right")
            angle = 30
        else:
            if(ob_cal >= 3):
                angle = int((mid - int(x_location))*-1)
            elif(last_ob_mode == "left_ob"):
                angle = -25
            elif(last_ob_mode == "right_ob"):
                angle = 20
            else:
                angle = -5

            # mid = 320
            angle = int((mid - int(x_location))*-1)


        # print(mode)
        
        # print(x_location)
        # if(mode == "left_drive"): 
        #     angle = -50
        # elif(mode == "right_drive"): 
        #     angle = 50
        # else:
        #     mid = 320
        #     angle = int((mid - int(x_location))*-1)


        # print(angle)
        
        # speed = max(5, 40 - abs(angle))
        speed = 4
        drive(angle, speed)

if __name__ == '__main__':
    start()
