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
k = 2
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
    global k, angle_cal, arData, l
    (ar_roll, ar_pitch, ar_yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
    angle = int(k * np.degrees(arData["DX"] - angle_cal)) - l

    return ar_roll, ar_pitch, ar_yaw, angle, arData["DX"], arData["DZ"]


# 눈을 왜 네모낳게 떠?

# 눈을 왜 세모낳게 떠??
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

        # # 초음파 정보와 arData의 정보를 조합하여 완전히 주차가 끝났는지를 판단.
        # if ultra.ultra_steer() == True and -0.5<= arData["DX"] <= 0.5 and 1 <= arData["DZ"] <= 1.5:
        #     print("break echo 1")
        #     break

        # # 뒤가 닿을 것 같을 때 앞으로 감.
        # elif ultra.ultra_steer() == True:
        #     print("break echo 2")
        #     sec = time.time()
        #     while time.time() - sec <= 0.5:
        #         drive(int(k * np.degrees(arData["DX"] - angle_cal)), 3)
        #     sec = time.time()
        #     while time.time() - sec <= 3:
        #         drive(0, 0)
        #     break

        if time.time() - sec_p <= 1.5:
            print("drive(50, -4)")
            drive(50, -4)
        elif time.time() - sec_p <= 1.5 + 0.5:
            print("drive(0, -4)")
            drive(0, -4)
        elif time.time() - sec_p <= 1.5 + 0.5 + 1.0:
            print("drive(50, -4)")
            drive(50, -4)
        # 여기서 else로 처리해줘도 될듯
        # 

    # 전체 다 했는데도 안되면 while-else문으로 해야할까


    # 주차 완료 후 빠져나오는 코드
    # sec = time.time()
    # while time.time() - sec <= 2:
    #     drive(-10, 4)

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
    global ar_id
    global last_x_location

    angle = 0
    DZ = 0

    p_parking_flag = 0
    
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
            elif ar_id == 1 and 0 < DZ <= 1:
                MODE = 1
            elif 3 <= ar_id <= 8 and 0 < DZ <= 1 and p_parking_flag != 1:
                MODE = 2
            elif ar_id == 9 and 0 < DZ <= 1:
                MODE = 3
            else:
                ar_id = None
                MODE = 0

        # 한번만 실행시키는거 flag 써서 해야할듯.
        # AR tag parking
        # 주차를 위해 일정 시간동안 앞으로 가는 코드. 한번만 실행되어야함. flag를 써도 됨.
        if MODE == 1 and 0 < DZ <= 0.7 and p_parking_flag == 0:
            sec = time.time()
            while time.time() - sec <= 3:
                # 오른쪽 초음파가 닿으면 break로 빠지는것도 괜찮을듯.
                drive(-5, 4)
            # 앞으로 가서 다른 ar 보고 주차 안하고 다른데로 갈 수도 있기 때문에,, 일부 후진까지만.
            sec = time.time()
            while time.time() - sec <= 2.5:
                drive(50, -4)
            sec = time.time()
            while time.time() - sec <= 1:
                drive(-5, -4)
            sec = time.time()
            while time.time() - sec <= 1.5:
                drive(-50, -4)
            
            angle, speed, parking = re_parking()
            drive(angle, speed)

            if(parking):
                p_parking_flag == 1

            MODE = 1

        # 계속 주차를 하며 실행되어야 할 코드.
        # 
        elif MODE == 1 and p_parking_flag >= 1:
            # 주차 멈추는 판정, 
            # 셋다 너무 가깝거나 말이 안되는경우 다시 주차. 
            # 아무것도 안잡히면 그냥 오른쪽으로 꺾어서 
            # main 비율(오른쪽뒤, 뒤 두개의 비율)로 조향갹을 가운데, 왼쪽
            print("echo")
            angry = ultra.ultra_steer()
            if angry == "stop":
                # 정지 후 앞의 주차 구역에 다시 주차 ar태그를 봐도 됨. 이후 일시 정지. 후 나가는 것 까지. MODE, ar_id 까지.
                print("stop")
                time.sleep(5)
            else:
                drive(angry, -4)

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
        # # MODE가 1인데 위에서 안걸리는 경우에 대한 처리. -> 안가는 이유.
        # # MODE에 대한 처리.
        # else:
        #     ar_id = None
        #     MODE = 0

        if MODE == 0:

            angle = int((320 - int(x_location))*-1)
            # speed = max(5, 40 - abs(angle))
            speed = 4
            drive(angle, speed)
        # # print(f"MODE: {MODE}")
        print(f"MODE: {MODE}")
        print(f"pPark: {p_parking_flag}")
        # print(f"ar_id: {ar_id}")
        # print(f"DZ: {DZ}")
        # print(f"angle: {angle}")
        


        # ar_id, MODE 원복.
        # ar_id = None
        # MODE = 0
        # DZ = 0

# ar_id, MODE 원복하는거 각 미션이 끝났을때 하기. 
# 미션을 끝내는 코드를 1) 미션의 코드에 넣을지, 2) 그냥 while 안에 넣을지.
# 후진주차 test
# 후진 안되고 초음파로 걸리는 issue.
# 초음파 callback 함수 바꾸기 숫자.
# 왜 차가 안갈까..? 차선 인식을 못하는 건가..? drive() 호출이 안되는건지 echo check
# 왜 안갈까..? 아무데에도 안걸리니까 안가는거 아닐까?? echo check(MODE, drive()함수 자체 호출 되는지.)
# 초음파로만 판단하도록 해도 될듯/..
if __name__ == '__main__':
    start()
