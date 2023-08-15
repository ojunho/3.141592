#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
import time

from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

from SlideWindow import SlideWindow
from Warper import Warper
from Ultrasonic import Ultrasonic

bridge = CvBridge()
warper = Warper()
slideWindow = SlideWindow()
ultra = Ultrasonic()
motor = None
flag = False

last_x_location = 320

cv_image = np.empty(shape=[0])

CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

MODE = 0

# for ar tag
arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
ar_roll, ar_pitch, ar_yaw = 0, 0, 0
k = 10
l = 30
angle_cal = 0.017
ar_id = None

def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

def ultra_callback(msg):
    global ultra
    ultra.ultra_data(msg.data)

# for ar tag
def ar_callback(msg):
    global ar_id, arData
    for i in msg.markers:
        ar_id = i.id
        pose = i.pose.pose
        arData["DX"] = pose.position.x
        arData["DY"] = pose.position.y
        arData["DZ"] = pose.position.z

        arData["AX"] = pose.orientation.x
        arData["AY"] = pose.orientation.y
        arData["AZ"] = pose.orientation.z
        arData["AW"] = pose.orientation.w

def id_back():
    global ar_id
    return ar_id

def ar_data():
    global k, l, angle_cal, arData
    (ar_roll, ar_pitch, ar_yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
    angle = int(k * np.degrees(arData["DX"] - angle_cal)) - l

    return ar_roll, ar_pitch, ar_yaw, angle, arData["DX"], arData["DZ"]

def ar_parking(x_location):

    global k
    global angle_cal
    global ultra
    global ar_id
    global arData
    global MODE


    print("AR Parking detected")

    # !!! 근데 이렇게 함수 안에서 주행을 하게 되면 while loop 안에서 실시간으로 바뀌는 정보를 못보는거 아닌가.. echo check
    # 만약 그렇다면 오차가 커지긴 할듯 차라리 angle = 0 으로 주행하는게 나을지도..
    # 일정시간동안 차선을 보고 주행을 함.
    angle = int((320 - int(x_location))*-1)
    # speed = max(5, 40 - abs(angle))
    speed = 5

    sec = time.time()
    while time.time() - sec <= 3:
        drive(angle, speed)

    # 후진으로 들어감 옆의 초음파를 확인하며 각을 꺾음 "오른쪽 뒷 초음파"
    
    sec_p = time.time()
    # 5초동안 돌아가는 while
    while time.time() - sec_p <= 5:

        # 초음파 정보와 arData의 정보를 조합하여 완전히 주차가 끝났는지를 판단.
        if ultra.ultra_steer() == True and -0.5<= arData["DX"] <= 0.5 and 1 <= arData["DZ"] <= 1.5:
            
            break

        # 뒤가 닿을 것 같을 때 앞으로 감.
        elif ultra.ultra_steer() == True:
            sec = time.time()
            while time.time() - sec <= 0.5:
                drive(int(k * np.degrees(arData["DX"] - angle_cal)), 3)
            sec = time.time()
            while time.time() - sec <= 3:
                drive(0, 0)
            break

        if time.time() - sec_p <= 1.5:
            drive(50, -3)
        elif time.time() - sec_p <= 1.5 + 0.5:
            drive(0, -3)
        elif time.time() - sec_p <= 1.5 + 0.5 + 1.0:
            drive(50, -3)
        # 여기서 else로 처리해줘도 될듯.

    # 전체 다 했는데도 안되면 while-else문으로 해야할까


    # 주차 완료 후 빠져나오는 코드
    sec = time.time()
    while time.time() - sec <= 2:
        drive(-10, 4)

    MODE = 0
    ar_id = None
    

def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()

    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)

def start():

    global motor
    global cv_image
    global angle_cal
    global MODE
    global flag
    global last_x_location
    global ar_id

    angle = 0
    DZ = 0
    
    rospy.init_node('ar_tag')
    
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_callback,queue_size= 1)
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback, queue_size = 1)

    print ("----- Xycar self driving -----")
    rospy.wait_for_message("/usb_cam/image_raw/", Image)

    while not rospy.is_shutdown():
        img = cv_image.copy()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)

        low_threshold = 70
        high_threshold = 210
        canny = cv2.Canny(blur_gray, low_threshold, high_threshold)
        kernel = np.ones((5, 5), np.uint8)
        close = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)
        warp_img = warper.warp(close)
        ret, thres_img = cv2.threshold(warp_img, 150, 255, cv2.THRESH_BINARY)

        cv2.imshow("original_img", img)
        cv2.imshow("processed_img", thres_img)
        cv2.waitKey(1)

        slide_img, x_location, _ = slideWindow.slidewindow(thres_img,  flag)
        x_loc = x_location

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

        ar_id = id_back()



        # MODE랑 ar_id를 어떻게 다시 돌리지..?


        # 일반 주행 코드의 위치의 중요성. MODE 먼저 바꾸는게 맞을듯.. MODE 적용 안하고 차선인식 결과 토대로 발행하면 안될듯.
        # MODE 확실히 관리하기. 멀리있는 ar tag 인식해버리면 답 없음.
        # 여기부터 판단 제어.
        # 차선인식 주행
        if MODE == 0:
            angle = int((320 - int(x_location))*-1)
            # speed = max(5, 40 - abs(angle))
            speed = 4
            drive(angle, speed)

        # id == 1 DZ >= 0.65
        # ar_tag 인식하고 angle 이상해지는것도 이거때문인가.. 아닌거같은데,,
        if ar_id != None:
            roll, pitch, yaw, angle, DX, DZ = ar_data()
            MODE = 1
        # 아니면 우회주행 MODE를 2로 해도 됨
        
        # ar 태그 주차 실시
        if MODE == 1 and ar_id == 1 and 0 < DZ <= 0.65:
            ar_parking(x_location)
        # ar 태그 우회 주행 실시 ar_data()로 계산한 angle로 drive()
        # MODE랑 ar_id 원복 해놓는거 여기서도 하는거 맞는지,, test 해보기
        elif MODE == 1 and 2<= ar_id <= 7:
            angle = angle
            speed = 4
            drive(angle, speed)
            MODE = 0
            ar_id = None
            # 끝나고 알아서 빠져 나오려나.. 빠져 나오면서 차선 알아서 잡으려나.. 

        
   
# 지금 하려고 하는거는 ar_tag 인식해서 주차도 하고, 우회 주행까지 하는 것.
# TODOLIST: MODE 원복 되는지 echo.
# MODE 원복하는것 조건으로 해야할듯.. 주행이 끝났을 시로,, 
# 원복 
# time.sleep() test 해보기. -> sleep 하면 이전 모터 발행 없어짐.. 
if __name__ == '__main__':
    start()
