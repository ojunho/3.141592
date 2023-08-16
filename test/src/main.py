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
from detection_msgs.msg import BoundingBoxes

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
k = 2
k1 = 1
k2 = 10
l = 30
l2 = 5
angle_cal = 0.017
angle_cal2 = 0.06
ar_id = None

# for bounding boxes
carLabel = "sonata"
dir = None
carCnt = 0

def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")


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
    global k, angle_cal, arData, l
    (ar_roll, ar_pitch, ar_yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
    angle = int(k * np.degrees(arData["DX"] - angle_cal)) - l

    return ar_roll, ar_pitch, ar_yaw, angle, arData["DX"], arData["DZ"]

def re_parking():
    global arData
    print(" AR PARKING DETECTED ")
    angle = arData["DX"] * 20
    speed = 0
    parking = False
    if(ultra.ultra_steer()):
        speed = -4
    else:
        speed = 4

    if(-0.05 < arData["DX"] < 0.05):
        parking = True
    
    return angle, speed, parking
        

def esc_p_parking():
    global MODE
    global ar_id
    global p_parking_flag


    drive(0, 0)
    time.sleep(3)
    sec = time.time()
    while time.time() - sec <= 0.9:
        drive(-5, -4)
    sec = time.time()
    while time.time() - sec <= 3:
        drive(-50, 4)
    sec = time.time()
    while time.time() - sec <= 1.5:
        drive(30, 4)
    MODE = 0
    ar_id = None
    p_parking_flag = 5


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
    global ar_id
    global last_x_location
    global k2
    global k
    global l2

    # count 변수들
    p_parking_cnt = 0

    p_parking_flag = 0

    # 평행주차 변수들
    # 수정을 하며 앞으로 갈 때

    # 수정을 하며 뒤로갈 때 
    back_first_half = 1
    back_second_half = 1

    # 처음 진입하며 들어가는 시간들
    p_parking_one_quarter_time = 3
    p_parking_two_quarter_time = 2.5
    p_parking_three_quarter_time = 0.7
    p_parking_four_quarter_time = 1.8

    angle = 0
    DZ = 0
    DX = 0


    
    rospy.init_node('ar_tag')
    
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_callback,queue_size= 1)
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback, queue_size = 1)
    rospy.Subscriber('/yolov5/detections', BoundingBoxes, boxes_callback)

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

        # # 차선주행 코드.
        # if MODE == 0:
        #     angle = int((320 - int(x_location))*-1)
        #     # speed = max(5, 40 - abs(angle))
        #     speed =0
        #     drive(angle, speed)

        # MODE에 대한 생각 
        # 0: 기본모드(차선인식 주행)
        # 1: ar_tag 평행주차
        # 2: ar_tag 우회주행
        # 3: 객체인식 T 주차
        #
        #
        #
        #
        ###################

        # set "MODE" to 1
        if ar_id != None:
            roll, pitch, yaw, angle, DX, DZ = ar_data()
            # 여기서 ar_id에 따라서 MODE를 부여하는 것도 괜찮을듯..
            if ar_id == 0:
                pass
            elif ar_id == 1 and 0 < DZ <= 0.7:
                MODE = 1
            elif 1 <= p_parking_flag <= 4:
                MODE = 1
            elif 3 <= ar_id <= 8 and 0 < DZ <= 1 and p_parking_flag >= 5:
                MODE = 2
            elif ar_id == 9 and 0 < DZ <= 1:
                MODE = 3
            else:
                ar_id = None
                MODE = 0

        # p_parking_flag: parallel parking, 즉 평행 주차를 했는지 안했는 지에 대한 여부
        # 0: 아직 안함
        # 1: 처음 실행되어 후진으로 진행(원트)
        # 2: 진행 하는 중임.(전진)
        # 3: 진행 하는 중임.(후진)
        # 4: 마지막 앞으로 가서 주차 하는 코드.
        # 5 이상

        # 평행 주차를 해야하는 코드 절대시간 sec_p를 선언함.
        if MODE == 1 and p_parking_flag == 0:
            sec_p = time.time()
            p_parking_flag = 1

        
        # 처음 실행되어 뒤로 가는 코드.
        if MODE == 1 and p_parking_flag == 1:
            
            # # 초음파 이슈로 일단 주석
            # # 너무 뒤로 갔을 때 예외
            # if ultra.ultra_steer() == True:
            #     p_parking_flag = 2
            #     sec_p = time.time()
            #     continue

                
            # 딱 멈추는 판정. 조건은 확인 해야함.
            if 0.051 <= DX <= 0.059 and 0.28 <= DZ <= 0.31:
                esc_p_parking()
                continue

            sec = time.time()
            if sec - sec_p <= p_parking_one_quarter_time:
                drive(-5, 4)
            elif sec - sec_p <= p_parking_one_quarter_time + p_parking_two_quarter_time:
                drive(50, -4)
            elif sec - sec_p <= p_parking_one_quarter_time + p_parking_two_quarter_time + p_parking_three_quarter_time:
                drive(-5, -4)
            elif sec - sec_p <= p_parking_one_quarter_time + p_parking_two_quarter_time + p_parking_three_quarter_time + p_parking_four_quarter_time:
                drive(-50, -4)
            else:
                p_parking_flag = 2
                sec_p = time.time()
        
        # 앞으로 가는 코드.
        if MODE == 1 and p_parking_flag == 2:
            sec = time.time()

            # 가라 주차 하는 코드.
            if p_parking_cnt == 3:
                p_parking_flag = 4
                sec_p = time.time()
                continue

            # 너무 앞으로 갔을 때 예외 처리.(DZ 너무 가깝거나 일정 시간 지나면)
            if 0 < DZ < 0.25 or sec - sec_p > 2:
                p_parking_flag = 3
                sec_p = time.time()
                continue

            # 딱 멈추는 판정. 조건은 확인 해야함.
            if 0.051 <= DX <= 0.059 and 0.28 <= DZ <= 0.31:
                esc_p_parking()
                continue
            
            if 0.051 <= DX <= 0.059:
                angle = -5
            else:
                angle = int(k1 * np.degrees(DX - angle_cal2)) - l2

            drive(angle, 4)

        # 다시 뒤로 가는 코드
        if MODE == 1 and p_parking_flag == 3:

            # # 너무 뒤로 갔을 때 예외
            # if ultra.ultra_steer() == True:
            #     p_parking_flag = 2
            #     sec_p = time.time()
            #     continue

            sec = time.time()
            if sec - sec_p > 2:
                p_parking_cnt += 1
                p_parking_flag = 2
                sec_p = time.time()
                continue

            # 딱 멈추는 판정. 조건은 확인 해야함.
            if 0.051 <= DX <= 0.059 and 0.28 <= DZ <= 0.31:
                esc_p_parking()
                continue


            if 0.051 <= DX <= 0.059:
                angle = -5
                drive(angle, -4)
            else:
                if sec - sec_p <= back_first_half:
                    angle = int(k2 * np.degrees(DX - angle_cal2)) - l2
                    drive(angle, -4)
                elif sec - sec_p <= back_first_half + back_second_half:
                    angle = int(-1*k2 * np.degrees(DX - angle_cal2)) - l2
                    drive(angle, -4)


        # 가라 주차 코드
        if MODE == 1 and p_parking_flag == 4:
            sec = time.time()
            if sec - sec_p <= 1.2:
                if 0.051 <= DX <= 0.059:
                    angle = -5
                else:
                    angle = int(k1 * np.degrees(DX - angle_cal2)) - l2
                drive(angle, 4)
            else:
                p_parking_flag = 5
                esc_p_parking()
                continue
        
        if MODE == 3 and dir == "left":
            pass
        
        elif MODE == 3 and dir == "right":
            pass




        # AR 우회주행
        if MODE == 2:
            angle = angle
            speed = 4
            # print("AR DRIVING")
            print(f"DX: {DX}   angle: {angle}")

            drive(angle, speed)
        
        # 우회주행을 하다가 마지막 AR태그의 정보를 이용해 빠져나오며 MODE 수정.
        if MODE == 2 and ar_id == 7 and 0 < DZ <= 0.65:
            sec = time.time()
            while time.time() - sec <= 1.5:
                drive(-50, 4)
            ar_id = None
            MODE = 0

        if MODE == 0:

            angle = int((320 - int(x_location))*-1)
            # speed = max(5, 40 - abs(angle))
            speed = 4
            drive(angle, speed)



        # print(f"MODE: {MODE}")
        # print(f"pPark: {p_parking_flag}")
        # print(f"ar_id: {ar_id}")

        # print(f"DX: {DX}")
        # print(f"DZ: {DZ}")
        # print(f"angle: {angle}")

if __name__ == '__main__':
    start()
