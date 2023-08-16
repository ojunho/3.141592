#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from detection_msgs.msg import BoundingBoxes

from ar_track_alvar_msgs.msg import _AlvarMarkers
from tf.transformations import euler_from_quaternion

import time

bridge = CvBridge()
cv_image = np.empty(shape=[0])
motor = None

# for ar tag
arData = {"DX":0.0, "DY":0.0, "Dz":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
ar_yaw = 0
k = 10
angle_cal = 0.017
ar_id = 0

# for bounding boxes
carLabel = "sonata"
dir = None
carCnt = 0

initialized = False
last_x_location = 320
x_loc = 0
flag = False

CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기



warper = Warper()
slideWindow = SlideWindow()

def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("mouse pos (x, y)", x, y)

def nothing(x):
    pass
#=============================================
#check the stopline fuction
def check_stopline() :
    img = cv_image.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    upper_white = np.array([255,255,255])
    lower_white = np.array([0,0,180])
    stop_line_img = cv2.inRange(hsv, lower_white, upper_white)
    cv2.imshow("HSV image for stopline detection", stop_line_img)
    cv2.waitKey(1)
    area = stop_line_img[380:420 , 200:440]
    stopline_count = cv2.countNonZero(area)
    #print("Stop Line Count = " + str(stopline_count))
    if stopline_count > 1700 :
        print("Stop Line Count = "+ str(stopline_count))
        return "stopLine"
    return False

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

# for ar tag
def ar_callback(msg):
    global ar_id
    for i in msg.markers:
        ar_id = i.id
        pose = i.pose.pose
        arData["DX"] = pose.position.x
        arData["DY"] = pose.position.y
        arData["Dz"] = pose.position.z

        arData["AX"] = pose.orientation.x
        arData["AY"] = pose.orientation.y
        arData["AZ"] = pose.orientation.z
        arData["AW"] = pose.orientation.w

def id_back():
    return ar_id

def ar_parking():
    print(ar_id)

#=============================================
# Bounding Boxes 토픽을 발행하는 함수
#=============================================
def boxes_callback(msg):

    global carLabel, dir, carCnt

    xSum = None

    for bounding_box in msg.bounding_boxes:
        if bounding_box.Class == carLabel:
            if bounding_box.probability >= 0.9:
                xSum = bounding_box.xmin + bounding_box.xmax
                carCnt += 1              
    
    if xSum != None and carCnt >= 30:
        if xSum <= 640:
            dir = "left"
        elif xSum > 640:
            dir = "right"
    
    print(carCnt)
    
    if dir != None:
        print(dir)


#=============================================
# 실질적인 메인 함수
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함.
#=============================================
def start():

    global motor, cv_image, initialized, flag, last_x_location, x_loc

    rospy.init_node('my_driver')

    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
    rospy.Subscriber('/yolov5/detections', BoundingBoxes, boxes_callback)

    print ("----- Xycar self driving -----")
    rospy.wait_for_message("/usb_cam/image_raw/", Image)

    while not rospy.is_shutdown():

        # 이미지 처리를 위해 카메라 원본 이미지를 img에 복사 저장한다.
        img = cv_image.copy()
        cv2.imshow("og", img)
        cv2.waitKey(1)

        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)

        low_threshold = 70
        high_threshold = 210
        canny = cv2.Canny(blur_gray, low_threshold, high_threshold)
        kernel = np.ones((5, 5), np.uint8)
        close = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)
        warp_img = warper.warp(close)
        ret, thres_img = cv2.threshold(warp_img, 150, 255, cv2.THRESH_BINARY)

        cv2.imshow("thres_img", thres_img)
        cv2.waitKey(1)

        slide_img, x_location, _ = slideWindow.slidewindow(thres_img,  flag)
        x_loc = x_location

        if x_location == None:
            flag = True
            x_location = last_x_location
        
        else:
            last_x_location = x_location
            flag = False
        
        cv2.imshow("window_view", slide_img)
        cv2.waitKey(1)

        angle = int((320 - int(x_location))*-1)
        
        angle = 0
        speed = 0
        drive(angle, speed)

        LANE = 1
        STOP_LINE = 2
        drive_mode = LANE

#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함
# start() 함수가 실질적인 메인 함수임.
#=============================================
if __name__ == '__main__':
    start()
