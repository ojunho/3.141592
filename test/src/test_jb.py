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
from obstacle_detector.msg import Obstacles

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
k1 = 3
k2 = 20
l = 30
l2 = 5
angle_cal = 0.017
angle_cal2 = 0.05
ar_id = None

# for bounding boxes
carLabel = "avante"
dir = None
carCnt = 0

# for 장애물
mode = None


# 라바콘 클래스
class Rabacon_drive:
    
    def __init__(self):
        rospy.Subscriber("raw_obstacles", Obstacles, self.Rabacon)
        self.angle = 0.0  # Initialize angle as a class attribute
        self.rabacon = False

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
            self.rabacon = True
        else:
            print("no_rabacon")
            self.rabacon = False


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
            if bounding_box.probability >= 0.85:
                xSum = bounding_box.xmin + bounding_box.xmax
                carCnt += 1              
    
    if xSum != None and carCnt >= 20:
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

# 장애물 피하기
def decision_Obstacles(data):
    global mode
    left_ob = []
    right_ob = []
    for i in data.circles:
        if i.center.x > -0.4 and i.center.x < -0.0:
            if i.center.y < 0 and i.center.y > -0.4:
                left_ob.append(i)
            elif i.center.y > 0 and i.center.y < 0.4:
                right_ob.append(i)
    
    if len(left_ob) > 0: #왼쪽 장애물
        print("left")
        mode = "left_ob"
    elif len(right_ob) > 0: #오른쪽 장애물
        print("right")
        mode = "right_ob"
    else:
        mode = "lane_drive"


def check_stopline(image):
    area = image[240:380, 185:380]
    stopline_count = cv2.countNonZero(area)
    # print(stopline_count)
    if stopline_count > 1650:
        return "StopLine"
    else:
        return "no"

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
    global mode


    # count 변수들
    p_parking_cnt = 0


    # flags
    t_parking_flag = 0
    # p와 t 사이 flag
    pt_flag = 0


    # 평행주차 변수들
    # 수정을 하며 앞으로 갈 때

    # 오른쪽 주차 시간
    # T 주차 변수들
    t_one_r = 0.9
    t_two_r = 3.2
    t_three_r = 0.8
    t_four_r = 3
    t_five_r = 1
    t_six_r = 1.5

    # 왼쪽 주차 시간
    # T 주차 변수들
    t_zero = 0.4
    t_one = 0.7
    t_two = 3.4
    t_three = 0.8
    t_four = 3
    t_five = 1.3
    t_six = 1.5

    angle = 0
    DZ = 0
    DX = 0

    # 장애물 변수들
    obstacle_cnt = 0
    direction = 0
    car_speed = 4

    MODE = 0
    
    rospy.init_node('ar_tag')
    
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_callback,queue_size= 1)
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback, queue_size = 1)
    rospy.Subscriber('/yolov5/detections', BoundingBoxes, boxes_callback)

    # 장애물
    rospy.Subscriber('raw_obstacles', Obstacles, decision_Obstacles)

    rabacon = Rabacon_drive()  # Create an instance of Rabacon_drive class

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
        check_stop = check_stopline(thres_img)


        # # 차선주행 코드.
        # if MODE == 0:
        #     angle = int((320 - int(x_location))*-1)
        #     # speed = max(5, 40 - abs(angle))
        #     speed =0
        #     drive(angle, speed)

        # MODE에 대한 생각 
        # 1: ar_tag 평행주차
        # 2: ar_tag 우회주행yuy
        # 3: T 주차 객체인식 전
        # 4: 장애물
        # 5: 횡단보도
        #
        #
        ###################

        # set "MODE" to 1
        # 객체인식 T 주차

        # t_parking_flag
        # 0: 아직 인식 못함.
        # 1: 객체 인식 진행중
        # 2: 객체인식 완료후 앞으로 가는 코드.
        # 3: DZ 일정 범위 이내에 들어와서 직진하는 코드.
        # 4: 주차 진행

        if ar_id != None:
            roll, pitch, yaw, angle, DX, DZ = ar_data()

        # 객체 인식을 시작한다는 뜻.
        if t_parking_flag == 0:
            t_parking_flag = 1

        if t_parking_flag == 1:
            if dir != None:
                t_parking_flag = 2
            else:
                drive(0, 0)

        # DZ 일정 이내에 들어오기 전까지는 차선을 보고 주행함.
        if t_parking_flag == 2:
            if 0 < DZ <= 0.6:
                print("직진 시작")
                # drive(0, 0)
                # time.sleep(3)
                t_parking_flag = 3
                sec_t = time.time()
            else:
                angle = int((320 - int(x_location))*-1)
                speed = 4
                drive(angle, speed)

        # 일단 직진 같이 함
        if t_parking_flag == 3:
            sec = time.time()
            # 직진 하는 시간
            if sec - sec_t <= 6.8:
                angle = int((320 - int(x_location))*-1)
                speed = 4
                drive(angle, speed)
            else:
                t_parking_flag = 4
                sec_t = time.time()


        # 왼쪽 주차 구역에 주차함
        if t_parking_flag == 4 and dir == "left":
            sec = time.time()
            if sec - sec_t <= t_zero:
                drive(-5, 4)
            if sec - sec_t <= t_zero + t_one:
                drive(35, 4)
            elif sec - sec_t <= t_zero +t_one + t_two:
                drive(-50, -4)
            elif sec - sec_t <= t_zero +t_one + t_two + t_three:
                drive(-5, -4)
            elif sec - sec_t <= t_zero + t_one + t_two + t_three + t_four:
                drive(-5, 0)
            elif sec - sec_t <= t_zero + t_one + t_two + t_three + t_four + t_five:
                drive(-5, 4)
            elif sec - sec_t <= t_zero + t_one + t_two + t_three + t_four + t_five + t_six:
                drive(-50, 4)
            else:
                t_parking_flag = 5

        # 오른쪽 주차 구역에 주차함
        elif t_parking_flag == 4 and dir == "right":
            sec = time.time()
            if sec - sec_t <= t_one:
                drive(-35, 4) 
            elif sec - sec_t <= t_one + t_two:
                drive(40, -4)
            elif sec - sec_t <= t_one + t_two + t_three: 
                drive(-5, -4)
            elif sec - sec_t <= t_one + t_two + t_three + t_four:
                drive(-5, 0)
            elif sec - sec_t <= t_one + t_two + t_three + t_four + t_five:
                drive(-5, 4)
            elif sec - sec_t <= t_one + t_two + t_three + t_four + t_five + t_six:
                drive(40, 4)
            else:
                t_parking_flag = 5


if __name__ == '__main__':
    start()
